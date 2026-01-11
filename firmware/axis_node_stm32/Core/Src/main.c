/**
 * @file main.c
 * @brief STM32G4 Axis Node Main Application
 * 
 * CANopen slave node for CNC axis control.
 * Implements CiA 402 drive profile for stepper motor control.
 * 
 * Features:
 * - CAN-FD at 500kbps/2Mbps
 * - Auto-assign node addressing
 * - Step/direction output for stepper driver
 * - Limit switch and home switch inputs
 * - Optional encoder feedback
 */

#include "main.h"
#include "canopen_slave.h"
#include "stepper_control.h"
#include <string.h>

// CANopen includes
#include "canopen_types.h"
#include "cia402.h"
#include "nmt.h"
#include "sdo.h"
#include "pdo.h"
#include "auto_assign.h"
#include "node_config.h"

// =============================================================================
// Peripheral Handles
// =============================================================================

FDCAN_HandleTypeDef hfdcan1;
TIM_HandleTypeDef   htim2;      // Step pulse generation
TIM_HandleTypeDef   htim3;      // Encoder input (optional)
TIM_HandleTypeDef   htim6;      // 1ms tick timer

// =============================================================================
// Global State
// =============================================================================

static volatile uint32_t g_tick_count = 0;
static volatile bool g_sync_received = false;

// Node configuration (stored in flash)
static full_node_config_t g_config;

// CiA 402 drive state
static cia402_drive_t g_drive;

// =============================================================================
// Forward Declarations
// =============================================================================

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void FDCAN1_Init(void);
static void TIM2_Init(void);
static void TIM6_Init(void);
static uint32_t get_unique_id(void);

// =============================================================================
// Main
// =============================================================================

int main(void)
{
    // HAL initialization
    HAL_Init();
    SystemClock_Config();
    
    // Peripheral initialization
    GPIO_Init();
    FDCAN1_Init();
    TIM2_Init();
    TIM6_Init();
    
    // Load or create default configuration
    uint32_t serial = get_unique_id();
    if (!node_config_load_flash(&g_config, (void*)0x0807F800)) {  // Last flash page
        // Create default stepper config
        node_config_default_stepper(&g_config, serial, AXIS_X);
    }
    
    // Initialize CiA 402 drive
    cia402_init(&g_drive);
    
    // Initialize CANopen slave
    canopen_slave_init(&g_config);
    
    // Start FDCAN
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    
    // Start timers
    HAL_TIM_Base_Start_IT(&htim6);  // 1ms tick
    
    // LED on - running
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    
    // Main loop
    while (1) {
        // Process CANopen stack
        canopen_slave_tick();
        
        // Process CiA 402 state machine
        if (g_sync_received) {
            g_sync_received = false;
            
            // Update drive state
            cia402_tick(&g_drive);
            
            // Execute motion if operational
            if (cia402_is_operational(&g_drive)) {
                stepper_control_update(g_drive.position_demand);
            }
        }
        
        // Read limit switches
        uint32_t inputs = 0;
        if (HAL_GPIO_ReadPin(LIMIT_POS_GPIO_Port, LIMIT_POS_Pin) == GPIO_PIN_SET)
            inputs |= (1 << 0);
        if (HAL_GPIO_ReadPin(LIMIT_NEG_GPIO_Port, LIMIT_NEG_Pin) == GPIO_PIN_SET)
            inputs |= (1 << 1);
        if (HAL_GPIO_ReadPin(HOME_SW_GPIO_Port, HOME_SW_Pin) == GPIO_PIN_SET)
            inputs |= (1 << 2);
        if (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_SET)
            inputs |= (1 << 3);
        
        g_drive.digital_inputs = inputs;
        
        // Check for faults
        if (inputs & (1 << 3)) {  // Driver fault
            cia402_trigger_fault(&g_drive, EMCY_DRIVE_FAULT);
        }
        
        // Update position from stepper controller
        g_drive.position_actual = stepper_control_get_position();
        g_drive.velocity_actual = stepper_control_get_velocity();
        
        // LED blink based on state
        static uint32_t last_blink = 0;
        uint32_t blink_rate = 1000;  // Default: 1Hz
        
        switch (canopen_slave_get_nmt_state()) {
            case NMT_STATE_PREOPERATIONAL:
                blink_rate = 500;
                break;
            case NMT_STATE_OPERATIONAL:
                blink_rate = 2000;
                break;
            case NMT_STATE_STOPPED:
                blink_rate = 250;
                break;
            default:
                break;
        }
        
        if (g_tick_count - last_blink >= blink_rate) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            last_blink = g_tick_count;
        }
    }
}

// =============================================================================
// System Clock Configuration (170 MHz from 8 MHz HSE)
// =============================================================================

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    // Configure voltage scaling
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    
    // Configure HSE and PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;     // 8MHz / 2 = 4MHz
    RCC_OscInitStruct.PLL.PLLN = 85;                 // 4MHz * 85 = 340MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;     // 340MHz / 2 = 170MHz
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    
    // Configure clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

// =============================================================================
// GPIO Initialization
// =============================================================================

static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // LED output
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
    
    // Step and Direction outputs
    GPIO_InitStruct.Pin = STEP_Pin | DIR_Pin | ENABLE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Limit switch inputs with pull-up
    GPIO_InitStruct.Pin = LIMIT_POS_Pin | LIMIT_NEG_Pin | HOME_SW_Pin | FAULT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Set initial states
    HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);  // Disabled
}

// =============================================================================
// FDCAN Initialization (CAN-FD: 500kbps/2Mbps)
// =============================================================================

static void FDCAN1_Init(void)
{
    __HAL_RCC_FDCAN_CLK_ENABLE();
    
    // Configure FDCAN GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = CAN_TX_Pin | CAN_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // FDCAN configuration
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;      // CAN-FD with bit rate switching
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = ENABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    
    // Nominal bit timing: 500 kbps @ 170 MHz
    // 170 MHz / 17 = 10 MHz time quantum
    // 10 MHz / (1 + 15 + 4) = 500 kbps
    hfdcan1.Init.NominalPrescaler = 17;
    hfdcan1.Init.NominalSyncJumpWidth = 4;
    hfdcan1.Init.NominalTimeSeg1 = 15;
    hfdcan1.Init.NominalTimeSeg2 = 4;
    
    // Data bit timing: 2 Mbps @ 170 MHz
    // 170 MHz / 5 = 34 MHz time quantum
    // 34 MHz / (1 + 13 + 3) = 2 Mbps
    hfdcan1.Init.DataPrescaler = 5;
    hfdcan1.Init.DataSyncJumpWidth = 3;
    hfdcan1.Init.DataTimeSeg1 = 13;
    hfdcan1.Init.DataTimeSeg2 = 3;
    
    hfdcan1.Init.StdFiltersNbr = 4;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    
    HAL_FDCAN_Init(&hfdcan1);
    
    // Configure filters
    FDCAN_FilterTypeDef filter = {0};
    
    // Filter 0: Accept all standard IDs (CANopen range)
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;   // Accept all
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    
    // Enable NVIC
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

// =============================================================================
// TIM2: Step pulse generation (used for timing)
// =============================================================================

static void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 170 - 1;         // 1 MHz tick
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;         // Free running
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
    HAL_TIM_Base_Start(&htim2);
}

// =============================================================================
// TIM6: 1ms tick timer
// =============================================================================

static void TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 170 - 1;         // 1 MHz tick
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000 - 1;           // 1 kHz interrupt
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);
    
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

// =============================================================================
// Get Unique Device ID
// =============================================================================

static uint32_t get_unique_id(void)
{
    // STM32 unique ID is at 0x1FFF7590
    uint32_t* uid = (uint32_t*)0x1FFF7590;
    return uid[0] ^ uid[1] ^ uid[2];
}

// =============================================================================
// Interrupt Handlers
// =============================================================================

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        g_tick_count++;
        canopen_slave_1ms_tick();
    }
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
    
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        // Convert to our frame format
        can_frame_t frame = {
            .id = rx_header.Identifier,
            .dlc = rx_header.DataLength >> 16,  // Convert DLC code to bytes
            .is_extended = (rx_header.IdType == FDCAN_EXTENDED_ID),
            .is_fd = (rx_header.FDFormat == FDCAN_FD_CAN),
            .is_rtr = (rx_header.RxFrameType == FDCAN_REMOTE_FRAME)
        };
        memcpy(frame.data, rx_data, frame.dlc);
        
        // Check for SYNC
        if (frame.id == CANOPEN_COB_ID(CANOPEN_FC_SYNC, 0)) {
            g_sync_received = true;
        }
        
        // Pass to CANopen stack
        canopen_slave_process_frame(&frame);
    }
}

// =============================================================================
// Error Handler
// =============================================================================

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for (volatile int i = 0; i < 1000000; i++);
    }
}

// =============================================================================
// HAL Tick (used by HAL_Delay)
// =============================================================================

uint32_t HAL_GetTick(void)
{
    return g_tick_count;
}
