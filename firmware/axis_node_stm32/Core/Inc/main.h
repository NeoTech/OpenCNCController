/**
 * @file main.h
 * @brief STM32G4 Axis Node Main Header
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

// =============================================================================
// Pin Definitions
// =============================================================================

// CAN-FD (FDCAN1)
#define CAN_TX_Pin          GPIO_PIN_12
#define CAN_TX_GPIO_Port    GPIOA
#define CAN_RX_Pin          GPIO_PIN_11
#define CAN_RX_GPIO_Port    GPIOA

// Stepper Motor Control
#define STEP_Pin            GPIO_PIN_0
#define STEP_GPIO_Port      GPIOA
#define DIR_Pin             GPIO_PIN_1
#define DIR_GPIO_Port       GPIOA
#define ENABLE_Pin          GPIO_PIN_2
#define ENABLE_GPIO_Port    GPIOA

// Limit Switches
#define LIMIT_POS_Pin       GPIO_PIN_3
#define LIMIT_POS_GPIO_Port GPIOA
#define LIMIT_NEG_Pin       GPIO_PIN_4
#define LIMIT_NEG_GPIO_Port GPIOA
#define HOME_SW_Pin         GPIO_PIN_5
#define HOME_SW_GPIO_Port   GPIOA

// Status LED
#define LED_Pin             GPIO_PIN_13
#define LED_GPIO_Port       GPIOC

// Fault Input (from motor driver)
#define FAULT_Pin           GPIO_PIN_6
#define FAULT_GPIO_Port     GPIOA

// Encoder (optional)
#define ENC_A_Pin           GPIO_PIN_6
#define ENC_A_GPIO_Port     GPIOB
#define ENC_B_Pin           GPIO_PIN_7
#define ENC_B_GPIO_Port     GPIOB

// =============================================================================
// Error Handler
// =============================================================================

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
