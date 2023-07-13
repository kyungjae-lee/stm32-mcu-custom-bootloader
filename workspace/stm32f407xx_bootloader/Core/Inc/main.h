/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>

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

/* Bootloader command handler functions */
void Bootloader_GetVer_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_GetHelp_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_GetCID_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_GetRDP_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_GoToAddr_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_EraseFlash_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_WriteMem_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_EnDisRWProtect_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_ReadMem_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_ReadProtectStatus_Cmd_Handler(uint8_t *pBLRxBuffer);
void Bootloader_ReadOTP_Cmd_Handler(uint8_t *pBLRxBuffer);

/* Bootloader helper functions */
void Bootloader_Tx_ACK(uint8_t cmdCode, uint8_t lenToFollow);
void Bootloader_Tx_NACK(void);
uint8_t Bootloader_Verify_CRC(uint8_t *pPacket, uint32_t len, uint32_t crcHost);
void Bootloader_UART_Write_Data(uint8_t *pBuffer, uint32_t len);
uint8_t Get_Bootloader_Version(void);
uint16_t Get_MCU_Chip_ID(void);
uint8_t Get_Flash_RDP_Level(void);
uint8_t Verify_Addr(uint32_t addr);


/* Bootloader function prototypes */
void Bootloader_UART_Read_Data(void);
void Bootloader_Jump_To_User_App(void);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

/* Bootloader version */
#define BL_VERSION				0x20

/* Bootloader command codes */
#define BL_GET_VER				0x51
#define BL_GET_HELP				0x52
#define BL_GET_CID				0x53
#define BL_GET_RDP_STATUS		0x54
#define BL_GO_TO_ADDR			0x55
#define BL_ERASE_FLASH			0x56
#define BL_WRITE_MEM			0x57
#define BL_ENDIS_RW_PROTECT		0x58
#define BL_READ_MEM				0x59
#define	BL_READ_PROTECT_STATUS	0x5A
#define BL_READ_OTP				0x5B

/* Bootloader ACK and NACK bytes */
#define BL_ACK					0xA5
#define BL_NACK					0x7F

/* CRC verification results */
#define CRC_VERIFICATION_SUCCESS	0
#define CRC_VERIFICATION_FAIL		1

#define ADDR_VALID 				0
#define ADDR_INVALID			1

/* Some start and end addresses of different memories of STM32F407xx MCU */
#define SRAM1_SIZE				112 * 1024 						/* 112 KB */
#define SRAM1_END				(SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE				16 * 1024 						/* 112 KB */
#define SRAM2_END				(SRAM2_BASE + SRAM1_SIZE)
#define FLASH_SIZE				1024 * 1024						/* 1 MB */
#define BKPSRAM_SIZE			4 * 1024						/* 4 KB */
#define BKPSRAM_END				(BKPSRAM_BASE + BKPSRAM_SIZE)


/* USER CODE END Private defines */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
