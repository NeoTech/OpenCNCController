/*
 * OpenCNC HAL Firmware - STM32F4/G4/H7
 * 
 * Main entry point for STM32 based motion controller.
 * Uses STM32 HAL/LL drivers with FreeRTOS.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#include "main.h"
#include "cmsis_os2.h"
#include "usbd_cdc_if.h"
#include <string.h>

#include "../common/hal_common.h"

// ============================================================================
// Hardware Defines
// ============================================================================

// Step/Direction pins (adjust for your board)
#define STEP_X_GPIO_Port    GPIOA
#define STEP_X_Pin          GPIO_PIN_0
#define DIR_X_GPIO_Port     GPIOA
#define DIR_X_Pin           GPIO_PIN_1

#define STEP_Y_GPIO_Port    GPIOA
#define STEP_Y_Pin          GPIO_PIN_2
#define DIR_Y_GPIO_Port     GPIOA
#define DIR_Y_Pin           GPIO_PIN_3

#define STEP_Z_GPIO_Port    GPIOA
#define STEP_Z_Pin          GPIO_PIN_4
#define DIR_Z_GPIO_Port     GPIOA
#define DIR_Z_Pin           GPIO_PIN_5

#define LIMIT_X_GPIO_Port   GPIOB
#define LIMIT_X_Pin         GPIO_PIN_0
#define LIMIT_Y_GPIO_Port   GPIOB
#define LIMIT_Y_Pin         GPIO_PIN_1
#define LIMIT_Z_GPIO_Port   GPIOB
#define LIMIT_Z_Pin         GPIO_PIN_2

#define PROBE_GPIO_Port     GPIOB
#define PROBE_Pin           GPIO_PIN_3
#define ESTOP_GPIO_Port     GPIOB
#define ESTOP_Pin           GPIO_PIN_4

#define SPINDLE_PWM_TIM     TIM3
#define SPINDLE_PWM_CH      TIM_CHANNEL_1
#define SPINDLE_EN_GPIO_Port GPIOC
#define SPINDLE_EN_Pin      GPIO_PIN_0

// Step generation timer
#define STEP_TIMER          TIM2
#define STEP_TIMER_IRQn     TIM2_IRQn
#define STEP_TIMER_FREQ     1000000  // 1 MHz

// ============================================================================
// Global State
// ============================================================================

static system_config_t g_config;
static realtime_status_t g_status;
static motion_segment_t g_motion_queue[MOTION_QUEUE_SIZE];
static volatile uint8_t g_queue_head = 0;
static volatile uint8_t g_queue_tail = 0;

static motion_segment_t g_current_segment;
static bool g_has_segment = false;
static steps_t g_current_position[MAX_AXES] = {0};
static uint32_t g_segment_tick = 0;

static TIM_HandleTypeDef htim_step;
static TIM_HandleTypeDef htim_spindle;

// FreeRTOS task handles
static osThreadId_t motion_task_handle;
static osThreadId_t status_task_handle;

// ============================================================================
// Step Timer ISR
// ============================================================================

void TIM2_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim_step, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim_step, TIM_FLAG_UPDATE);
        
        if (!g_has_segment) {
            return;
        }
        
        // Bresenham step generation
        g_segment_tick++;
        
        uint32_t total_ticks = g_current_segment.accel_ticks + 
                               g_current_segment.cruise_ticks + 
                               g_current_segment.decel_ticks;
        
        if (g_segment_tick >= total_ticks) {
            for (int i = 0; i < MAX_AXES; i++) {
                g_current_position[i] = g_current_segment.target[i];
            }
            g_has_segment = false;
        }
    }
}

// ============================================================================
// GPIO Configuration
// ============================================================================

static void configure_gpio(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // Step/Dir pins as outputs
    GPIO_InitStruct.Pin = STEP_X_Pin | DIR_X_Pin | STEP_Y_Pin | DIR_Y_Pin | 
                          STEP_Z_Pin | DIR_Z_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Limit/probe pins as inputs with pull-up
    GPIO_InitStruct.Pin = LIMIT_X_Pin | LIMIT_Y_Pin | LIMIT_Z_Pin | 
                          PROBE_Pin | ESTOP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Spindle enable
    GPIO_InitStruct.Pin = SPINDLE_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SPINDLE_EN_GPIO_Port, &GPIO_InitStruct);
}

// ============================================================================
// Timer Configuration
// ============================================================================

static void configure_step_timer(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    htim_step.Instance = STEP_TIMER;
    htim_step.Init.Prescaler = (SystemCoreClock / STEP_TIMER_FREQ) - 1;
    htim_step.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_step.Init.Period = 10;  // Initial period
    htim_step.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_step.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    HAL_TIM_Base_Init(&htim_step);
    
    HAL_NVIC_SetPriority(STEP_TIMER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(STEP_TIMER_IRQn);
}

static void configure_spindle_pwm(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    htim_spindle.Instance = SPINDLE_PWM_TIM;
    htim_spindle.Init.Prescaler = (SystemCoreClock / 1000000) - 1;  // 1 MHz
    htim_spindle.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_spindle.Init.Period = 200;  // 5 kHz PWM
    htim_spindle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    HAL_TIM_PWM_Init(&htim_spindle);
    
    TIM_OC_InitTypeDef oc_config = {0};
    oc_config.OCMode = TIM_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    
    HAL_TIM_PWM_ConfigChannel(&htim_spindle, &oc_config, SPINDLE_PWM_CH);
    HAL_TIM_PWM_Start(&htim_spindle, SPINDLE_PWM_CH);
}

// ============================================================================
// HAL Implementation
// ============================================================================

void hal_init(void) {
    memset(&g_config, 0, sizeof(g_config));
    g_config.num_axes = 3;
    
    for (int i = 0; i < 3; i++) {
        g_config.axes[i].enabled = true;
        g_config.axes[i].steps_per_mm = 800.0;
        g_config.axes[i].max_velocity = 5000.0;
        g_config.axes[i].max_acceleration = 500.0;
        g_config.axes[i].max_travel = 200.0;
    }
    
    configure_gpio();
    configure_step_timer();
    configure_spindle_pwm();
    
    g_status.state = STATE_IDLE;
    g_status.alarm = ALARM_NONE;
}

bool hal_motion_queue_push(const motion_segment_t* segment) {
    uint8_t next_head = (g_queue_head + 1) % MOTION_QUEUE_SIZE;
    if (next_head == g_queue_tail) {
        return false;
    }
    
    memcpy(&g_motion_queue[g_queue_head], segment, sizeof(motion_segment_t));
    g_queue_head = next_head;
    return true;
}

bool hal_motion_queue_pop(motion_segment_t* segment) {
    if (g_queue_head == g_queue_tail) {
        return false;
    }
    
    memcpy(segment, &g_motion_queue[g_queue_tail], sizeof(motion_segment_t));
    g_queue_tail = (g_queue_tail + 1) % MOTION_QUEUE_SIZE;
    return true;
}

uint8_t hal_motion_queue_depth(void) {
    if (g_queue_head >= g_queue_tail) {
        return g_queue_head - g_queue_tail;
    }
    return MOTION_QUEUE_SIZE - g_queue_tail + g_queue_head;
}

void hal_motion_queue_clear(void) {
    g_queue_head = 0;
    g_queue_tail = 0;
}

void hal_motion_start(void) {
    if (g_status.state == STATE_IDLE || g_status.state == STATE_PAUSED) {
        g_status.state = STATE_RUNNING;
        __HAL_TIM_ENABLE_IT(&htim_step, TIM_IT_UPDATE);
        HAL_TIM_Base_Start(&htim_step);
    }
}

void hal_motion_pause(void) {
    if (g_status.state == STATE_RUNNING) {
        g_status.state = STATE_PAUSED;
        HAL_TIM_Base_Stop(&htim_step);
    }
}

void hal_motion_resume(void) {
    if (g_status.state == STATE_PAUSED) {
        g_status.state = STATE_RUNNING;
        HAL_TIM_Base_Start(&htim_step);
    }
}

void hal_motion_stop(void) {
    g_status.state = STATE_IDLE;
    HAL_TIM_Base_Stop(&htim_step);
    hal_motion_queue_clear();
    g_has_segment = false;
}

void hal_motion_estop(void) {
    g_status.state = STATE_ESTOP;
    g_status.alarm = ALARM_ESTOP;
    HAL_TIM_Base_Stop(&htim_step);
    hal_motion_queue_clear();
    g_has_segment = false;
    
    HAL_GPIO_WritePin(SPINDLE_EN_GPIO_Port, SPINDLE_EN_Pin, GPIO_PIN_RESET);
}

void hal_spindle_set(bool on, bool cw, uint16_t rpm) {
    (void)cw;  // Direction control not implemented
    
    if (on) {
        uint32_t duty = (rpm * 200) / g_config.spindle_max_rpm;
        __HAL_TIM_SET_COMPARE(&htim_spindle, SPINDLE_PWM_CH, duty);
        HAL_GPIO_WritePin(SPINDLE_EN_GPIO_Port, SPINDLE_EN_Pin, GPIO_PIN_SET);
        
        g_status.spindle_on = true;
        g_status.spindle_cw = cw;
        g_status.spindle_rpm = rpm;
    } else {
        hal_spindle_stop();
    }
}

void hal_spindle_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim_spindle, SPINDLE_PWM_CH, 0);
    HAL_GPIO_WritePin(SPINDLE_EN_GPIO_Port, SPINDLE_EN_Pin, GPIO_PIN_RESET);
    
    g_status.spindle_on = false;
    g_status.spindle_rpm = 0;
}

void hal_get_status(realtime_status_t* status) {
    memcpy(status, &g_status, sizeof(realtime_status_t));
    
    for (int i = 0; i < MAX_AXES; i++) {
        status->position[i] = g_current_position[i];
    }
    
    status->limit_state = 0;
    if (HAL_GPIO_ReadPin(LIMIT_X_GPIO_Port, LIMIT_X_Pin) == GPIO_PIN_RESET)
        status->limit_state |= 0x01;
    if (HAL_GPIO_ReadPin(LIMIT_Y_GPIO_Port, LIMIT_Y_Pin) == GPIO_PIN_RESET)
        status->limit_state |= 0x02;
    if (HAL_GPIO_ReadPin(LIMIT_Z_GPIO_Port, LIMIT_Z_Pin) == GPIO_PIN_RESET)
        status->limit_state |= 0x04;
    
    status->probe_triggered = (HAL_GPIO_ReadPin(PROBE_GPIO_Port, PROBE_Pin) == GPIO_PIN_RESET);
    status->queue_depth = hal_motion_queue_depth();
}

void hal_get_position(steps_t* position) {
    for (int i = 0; i < MAX_AXES; i++) {
        position[i] = g_current_position[i];
    }
}

// ============================================================================
// FreeRTOS Tasks
// ============================================================================

static void motion_task_func(void *argument) {
    (void)argument;
    
    while (1) {
        if (g_status.state == STATE_RUNNING && !g_has_segment) {
            if (hal_motion_queue_pop(&g_current_segment)) {
                g_has_segment = true;
                g_segment_tick = 0;
                g_status.current_segment_id = g_current_segment.id;
            } else {
                g_status.state = STATE_IDLE;
                HAL_TIM_Base_Stop(&htim_step);
            }
        }
        
        osDelay(1);
    }
}

static void status_task_func(void *argument) {
    (void)argument;
    
    while (1) {
        hal_send_status();
        osDelay(1000 / STATUS_UPDATE_HZ);
    }
}

void hal_send_status(void) {
    realtime_status_t status;
    hal_get_status(&status);
    
    uint8_t packet[64];
    size_t len = 0;
    
    packet[len++] = 0xAA;
    packet[len++] = 0xA2;  // RT_STATUS
    packet[len++] = 0x00;
    packet[len++] = sizeof(realtime_status_t);
    
    memcpy(&packet[len], &status, sizeof(realtime_status_t));
    len += sizeof(realtime_status_t);
    
    packet[len++] = 0x00;  // CRC placeholder
    packet[len++] = 0x00;
    packet[len++] = 0x55;
    
    CDC_Transmit_FS(packet, len);
}

// ============================================================================
// USB CDC Receive Callback
// ============================================================================

void HAL_USB_CDC_RxCallback(uint8_t* buf, uint32_t len) {
    // Process incoming packets
    // TODO: Implement protocol packet parsing
    (void)buf;
    (void)len;
}

// ============================================================================
// Main Entry
// ============================================================================

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    hal_init();
    
    // USB initialization handled by CubeMX
    
    osKernelInitialize();
    
    const osThreadAttr_t motion_attr = {
        .name = "motion",
        .priority = osPriorityRealtime,
        .stack_size = 1024
    };
    motion_task_handle = osThreadNew(motion_task_func, NULL, &motion_attr);
    
    const osThreadAttr_t status_attr = {
        .name = "status",
        .priority = osPriorityNormal,
        .stack_size = 1024
    };
    status_task_handle = osThreadNew(status_task_func, NULL, &status_attr);
    
    osKernelStart();
    
    while (1) {
        // Should never reach here
    }
}

// Weak placeholder for CubeMX
__weak void SystemClock_Config(void) {
    // Configure in STM32CubeMX or manually
}
