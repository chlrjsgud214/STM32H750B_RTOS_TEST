/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */

  /* USER CODE END ET */

  /* Exported constants --------------------------------------------------------*/
  /* USER CODE BEGIN EC */

  /* USER CODE END EC */

  /* Exported macro ------------------------------------------------------------*/
  /* USER CODE BEGIN EM */

  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FDCAN2_RX_Pin GPIO_PIN_5
#define FDCAN2_RX_GPIO_Port GPIOB
#define SDIO1_D2_Pin GPIO_PIN_10
#define SDIO1_D2_GPIO_Port GPIOC
#define MII_TX_EN_Pin GPIO_PIN_11
#define MII_TX_EN_GPIO_Port GPIOG
#define LCD_DISP_Pin GPIO_PIN_15
#define LCD_DISP_GPIO_Port GPIOJ
#define SDIO1_D3_Pin GPIO_PIN_11
#define SDIO1_D3_GPIO_Port GPIOC
#define FDCAN1_RX_Pin GPIO_PIN_14
#define FDCAN1_RX_GPIO_Port GPIOH
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define MII_TXD3_Pin GPIO_PIN_2
#define MII_TXD3_GPIO_Port GPIOE
#define MII_TXD1_Pin GPIO_PIN_12
#define MII_TXD1_GPIO_Port GPIOG
#define LCD_DISPD7_Pin GPIO_PIN_7
#define LCD_DISPD7_GPIO_Port GPIOD
#define SDIO1_CK_Pin GPIO_PIN_12
#define SDIO1_CK_GPIO_Port GPIOC
#define SDIO1_D5_Pin GPIO_PIN_9
#define SDIO1_D5_GPIO_Port GPIOB
#define SDIO1_D4_Pin GPIO_PIN_8
#define SDIO1_D4_GPIO_Port GPIOB
#define MII_TXD0_Pin GPIO_PIN_13
#define MII_TXD0_GPIO_Port GPIOG
#define SDIO1_CMD_Pin GPIO_PIN_2
#define SDIO1_CMD_GPIO_Port GPIOD
#define USB_OTG_FS2_ID_Pin GPIO_PIN_10
#define USB_OTG_FS2_ID_GPIO_Port GPIOA
#define VBUS_FS2_Pin GPIO_PIN_9
#define VBUS_FS2_GPIO_Port GPIOA
#define FDCAN1_TX_Pin GPIO_PIN_13
#define FDCAN1_TX_GPIO_Port GPIOH
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define SDIO1_D0_Pin GPIO_PIN_8
#define SDIO1_D0_GPIO_Port GPIOC
#define SDIO1_D1_Pin GPIO_PIN_9
#define SDIO1_D1_GPIO_Port GPIOC
#define USB_OTG_FS2_P_Pin GPIO_PIN_12
#define USB_OTG_FS2_P_GPIO_Port GPIOA
#define USB_OTG_FS2_N_Pin GPIO_PIN_11
#define USB_OTG_FS2_N_GPIO_Port GPIOA
#define MII_RX_ER_Pin GPIO_PIN_10
#define MII_RX_ER_GPIO_Port GPIOI
#define SDIO1_D7_Pin GPIO_PIN_7
#define SDIO1_D7_GPIO_Port GPIOC
#define SDIO1_D6_Pin GPIO_PIN_6
#define SDIO1_D6_GPIO_Port GPIOC
#define audio_Int_Pin GPIO_PIN_7
#define audio_Int_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOI
#define LCD_INT_Pin GPIO_PIN_2
#define LCD_INT_GPIO_Port GPIOG
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOK
#define MII_MDC_Pin GPIO_PIN_1
#define MII_MDC_GPIO_Port GPIOC
#define MII_TXD2_Pin GPIO_PIN_2
#define MII_TXD2_GPIO_Port GPIOC
#define MII_TX_CLK_Pin GPIO_PIN_3
#define MII_TX_CLK_GPIO_Port GPIOC
#define MII_CRS_Pin GPIO_PIN_2
#define MII_CRS_GPIO_Port GPIOH
#define MII_MDIO_Pin GPIO_PIN_2
#define MII_MDIO_GPIO_Port GPIOA
#define MII_RX_CLK_Pin GPIO_PIN_1
#define MII_RX_CLK_GPIO_Port GPIOA
#define MII_COL_Pin GPIO_PIN_3
#define MII_COL_GPIO_Port GPIOH
#define VCP_TX_Pin GPIO_PIN_10
#define VCP_TX_GPIO_Port GPIOB
#define VCP_RX_Pin GPIO_PIN_11
#define VCP_RX_GPIO_Port GPIOB
#define OTG_FS2_OverCurrent_Pin GPIO_PIN_11
#define OTG_FS2_OverCurrent_GPIO_Port GPIOH
#define MII_RX_DV_Pin GPIO_PIN_7
#define MII_RX_DV_GPIO_Port GPIOA
#define MII_TX_ER_nINT_Pin GPIO_PIN_2
#define MII_TX_ER_nINT_GPIO_Port GPIOB
#define OTG_FS2_PSO_Pin GPIO_PIN_5
#define OTG_FS2_PSO_GPIO_Port GPIOA
#define MII_RXD0_Pin GPIO_PIN_4
#define MII_RXD0_GPIO_Port GPIOC
#define MII_RX_D3_Pin GPIO_PIN_1
#define MII_RX_D3_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_2
#define LD1_GPIO_Port GPIOJ
#define LCD_RST_Pin GPIO_PIN_12
#define LCD_RST_GPIO_Port GPIOB
#define MII_RXD1_Pin GPIO_PIN_5
#define MII_RXD1_GPIO_Port GPIOC
#define MII_RX_D2_Pin GPIO_PIN_0
#define MII_RX_D2_GPIO_Port GPIOB
#define FDCAN2_TX_Pin GPIO_PIN_13
#define FDCAN2_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define EVENT_FLAG_UART_RX_DATA_READY (1 << 0)
#define EVENT_FLAG_UART_TX_DATA_READY (1 << 1)
#define EVENT_FLAG_SYSTEM_RESET       (1 << 2)
#define EVENT_FLAG_UART_RX_BUFFER_OVERFLOW (1 << 3)
#define EVENT_FLAG_BUFFER_CUT_ENABLE (1 << 4)

#define IDLE_TASK_STACK_SIZE (128 * 4) // Stack은 word(4byte) 단위로 계산
#define TmrSvc_TASK_STACK_SIZE (256 * 4)

#define UART_RX_TASK_STACK_SIZE (512 * 4)
#define UART_TX_TASK_STACK_SIZE (512 * 4)
#define TASK_1MS_STACK_SIZE (256 * 4)
#define TASK_100MS_STACK_SIZE (256 * 4)
#define TASK_500MS_STACK_SIZE (128 * 4)
#define TASK_1000MS_STACK_SIZE (128 * 4)
#define TASK_MONITOR_STACK_SIZE (512 * 4)

#define TASK_MONITOR_PERIOD     2000    // 태스크 모니터링 주기 (ms)
#define TASK_MONITOR_THRESHOLD  5000    // 태스크 응답 없음 판단 임계값 (ms)
#define SYSTEM_RESET_DELAY      1000    // 시스템 리셋 전 대기 시간 (ms)

#define TOTAL_STACK_SIZE UART_RX_TASK_STACK_SIZE +     \
                             UART_TX_TASK_STACK_SIZE + \
                             TASK_100MS_STACK_SIZE +   \
                             TASK_500MS_STACK_SIZE +   \
                             TASK_1000MS_STACK_SIZE +  \
                             TASK_MONITOR_STACK_SIZE + \
                             IDLE_TASK_STACK_SIZE +    \
                             TmrSvc_TASK_STACK_SIZE +  \
                             TASK_1MS_STACK_SIZE

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
