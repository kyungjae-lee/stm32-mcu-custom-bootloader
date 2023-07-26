/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdarg.h>
#include <string.h>
#include <stdio.h>	/* vsprintf */

#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BL_DEBUG_MSG_EN
#define D_UART					&huart3		/* Debug USART */
#define C_UART					&huart2		/* Virtual COM Port USART */
#define BL_RX_LEN				200
#define MAX_NUM_OF_SECTORS		12			/* STM32F407xx MCUs specific */

/* Flash option control register */
#define FLASH_OPTCR				(*(uint32_t volatile *)0x40023C14)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

char data[] = "Message from bootloader\r\n";
uint8_t blRxBuffer[BL_RX_LEN];

uint8_t supportedCmds[] = {
	BL_GET_VER,
	BL_GET_HELP,
	BL_GET_CID,
	BL_GO_TO_ADDR,
	BL_ERASE_FLASH,
	BL_READ_MEM,
	BL_WRITE_MEM,
	BL_GET_RDP_LEVEL,
	BL_SET_RDP_LEVEL,
	BL_ENABLE_WRP,
	BL_DISABLE_WRP,
	BL_GET_WRP_STATUS,
	BL_READ_OTP,
};
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static void Print_Msg(char *format, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
	{
		Print_Msg("BL_DEBUG_MSG: Button pressed ... going to BL mode\n\r");
		
		/* Continue in the bootloader mode */
		Bootloader_UART_Read_Data();
	}
	else
	{
		Print_Msg("BL_DEBUG_MSG:: Button not pressed ... executing user application\n\r");
		Bootloader_Jump_To_User_App();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void Bootloader_UART_Read_Data(void)
{
	uint8_t rcvLen = 0;
	
	while (1)
	{
		memset(blRxBuffer, 0, 200);
		
		/* Read and decode the command coming from the host */
		
		/* First, read the first byte (i.e., the "Length to follow" field ) */
		HAL_UART_Receive(C_UART, blRxBuffer, 1, HAL_MAX_DELAY);
		rcvLen = blRxBuffer[0];	/* Number of the remaining bytes to read */
		
		/* Second, read the remaining bytes, first of which is the "Command code" */
		HAL_UART_Receive(C_UART, &blRxBuffer[1], rcvLen, HAL_MAX_DELAY);
		
		/* Decode the command code */
		switch (blRxBuffer[1])
		{
			case BL_GET_VER:
				Bootloader_GetVer_Cmd_Handler(blRxBuffer);
				break;
			case BL_GET_HELP:
				Bootloader_GetHelp_Cmd_Handler(blRxBuffer);
				break;
			case BL_GET_CID:
				Bootloader_GetCID_Cmd_Handler(blRxBuffer);
				break;
			case BL_GO_TO_ADDR:
				Bootloader_GoToAddr_Cmd_Handler(blRxBuffer);
				break;
			case BL_ERASE_FLASH:
				Bootloader_EraseFlash_Cmd_Handler(blRxBuffer);
				break;
			case BL_READ_MEM:
				Bootloader_ReadMem_Cmd_Handler(blRxBuffer);
				break;
			case BL_WRITE_MEM:
				Bootloader_WriteMem_Cmd_Handler(blRxBuffer);
				break;
			case BL_GET_RDP_LEVEL:
				Bootloader_GetRDPLevel_Cmd_Handler(blRxBuffer);
				break;
			case BL_SET_RDP_LEVEL:
				Bootloader_GetRDPLevel_Cmd_Handler(blRxBuffer);
				break;
			case BL_ENABLE_WRP:
				Bootloader_EnableWRP_Cmd_Handler(blRxBuffer);
				break;
			case BL_DISABLE_WRP:
				Bootloader_DisableWRP_Cmd_Handler(blRxBuffer);
				break;
			case BL_GET_WRP_STATUS:
				Bootloader_GetWRPStatus_Cmd_Handler(blRxBuffer);
				break;
			case BL_READ_OTP:
				Bootloader_ReadOTP_Cmd_Handler(blRxBuffer);
				break;
			default:
				Print_Msg("BL_DEBUG_MSG: Invalid command code received from the host\n");
				break;
		}
	}
}

/* Jump to the user application.
 * Here, we are assuming that the FLASH_SECTOR2_BASE_ADDRESS is where the user
 * application is stored
 * Note: Make sure to uncomment 'USER_VECT_TAB_ADDRESS' macro in the 
 *       system_stm32f4xx.c file since we are relocating the vector table.
 */
void Bootloader_Jump_To_User_App(void)
{
	/* Function pointer to hold the address ofthe reset handler of the user app */
	void (*appResetHandler)(void);
	
	Print_Msg("BL_DEBUG_MSG: Bootloader_Jump_To_User_App()\n");
	
	/* 1. Configure the MSP by reading the value from the base address of the sector 2.
	 *    The very first element of the vector table holds the initial value of the msp.
	 */
	uint32_t msp = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	Print_Msg("BL_DEBUG_MSG: MSP: %#x\n", msp);
	
	/* This function comes from CMSIS */
	__set_MSP(msp);
	
	//SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;
	
	/* 2. Fetch the address of the Reset_Handler() of the user application from
	 *    the location FLASH_SECTOR2_BASE_ADDRESS+4
	 */
	uint32_t resetHandlerAddr = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);
	
	appResetHandler = (void *)resetHandlerAddr;
	
	Print_Msg("BL_DEBUG_MSG: User application Reset_Handler() address: %#x\n", appResetHandler);
	
	/* Jumpt to the Reset_Handler() of the user application */
	appResetHandler();
}


/* Print a formatted string to console over USART for debugging */
void Print_Msg(char *format, ...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
	
	/* Extract the argument list using VA APIs */
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
#endif
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Implementation of the bootloader command handler functions ----------------*/

/**
 * Bootloader_GetVer_Cmd_Handler()
 * Brief	: Handles BL_GET_VER command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_GET_VER command is used to read the bootloader version from the
 *  		  MCU.
 */
void Bootloader_GetVer_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint8_t blVersion;
	
	Print_Msg("BL_DEBUG_MSG: bootloader_getver_cmd_handler()\n");
	
	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));
	
	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 1);
		blVersion = Get_Bootloader_Version();
		Print_Msg("BL_DEBUG_MSG: BL_VER: %d %#x\n", blVersion, blVersion);
		
		/* Send response to the host */
		Bootloader_UART_Write_Data(&blVersion, 1);
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_GetVer_Cmd_Handler */

/**
 * Bootloader_GetHelp_Cmd_Handler()
 * Brief	: Handles BL_GET_HELP command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_GET_HELP command is used to retrieve all the commands that are 
 * 			  supported by the bootloader.
 */
void Bootloader_GetHelp_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	Print_Msg("BL_DEBUG_MSG: Bootloader_GetHelp_Cmd_Handler()\n");
	
	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], sizeof(supportedCmds));
		
		/* Send response to the host */
		Bootloader_UART_Write_Data(supportedCmds, sizeof(supportedCmds));
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_GetHelp_Cmd_Handler */

/**
 * Bootloader_GetCID_Cmd_Handler()
 * Brief	: Handles BL_GET_CID command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_GET_CID command is used to read the MCU chip identification
 *			  number.
 *			  This function can be updated to read the Revision Identifier
 *			  (REV_ID) as well.
 */
void Bootloader_GetCID_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	Print_Msg("BL_DEBUG_MSG: Bootloader_GetCID_Cmd_Handler()\n");
	
	uint16_t cid;

	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 2);
		
		cid = Get_MCU_Chip_ID();
		Print_Msg("BL_DEBUG_MSG: MCU Chip ID: %d %#x\n", cid, cid);
		
		/* Send response to the host */
		Bootloader_UART_Write_Data((uint8_t *)&cid, 2);
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_GetCID_Cmd_Handler */

/**
 * Bootloader_GoToAddr_Cmd_Handler()
 * Brief	: Handles BL_GO_TO_ADDR command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_GO_TO_ADDR command is used to jump bootloader to specified 
 *			  address.
 *			  A good way to test this functionality is to set the target 
 *			  address to the reset handler of the user application and see if
 *			  the program successfully jumps to the user application. (Reset 
 *			  handler is the entry point to the user application)
 *			  When entering the target address to the host application, 
 *			  make sure to account for the T-bit setting. For example, 
 *			  If the address of the user application's reset handler is 
 *			  0x08008229, then enter 0x08008228.
 */
void Bootloader_GoToAddr_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint32_t targetAddr = 0;
	uint8_t addrValid = ADDR_VALID;
	uint8_t addrInvalid = ADDR_INVALID;
	
	Print_Msg("BL_DEBUG_MSG: Bootloader_GoToAddr_Cmd_Handler\n");
	
	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 1);
		
		targetAddr = *((uint32_t *)&pBLRxBuffer[2]);
		Print_Msg("BL_DEBUG_MSG: Target address to jump to: %#x\n", targetAddr);
		
		if (Verify_Addr(targetAddr) == ADDR_VALID)
		{
			/* Notify the host that the address is valid */
			Bootloader_UART_Write_Data(&addrValid, 1);
			
			/* Jump to addr.
			 * Not executing the following line will trigger the hardfault
			 * exception for ARM Cortex-M processors.
			 * https://www.youtube.com/watch?v=VX_12SjnNhY
			 */
			targetAddr += 1;	/* Set T-bit to 1 */
			
			void (*jumpToTargetAddr)(void) = (void *)targetAddr;
			
			Print_Msg("BL_DEBUG_MSG: Jumping to the target address\n");
			
			jumpToTargetAddr();
		}
		else
		{
			Print_Msg("BL_DEBUG_MSG: Invalid target address\n");
			Bootloader_UART_Write_Data(&addrInvalid, 1);
		}
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_GoToAddr_Cmd_Handler */

/**
 * Bootloader_EraseFlash_Cmd_Handler()
 * Brief	: Handles BL_ERASE_FLASH command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_ERASE_FLASH command is used to mass-erase or sector-erase the
 *			  user Flash. Erasing a byte of memory space means setting it to 0xFF.
 *			  Check out the 'Erase and program operations' section of the MCU 
 *			  reference manual for more details about sector-erase and mass-erase.
 *			  This function will use Flash driver APIs to implement the erasing 
 *			  functionality. (stn32f4xx_hal_flash_ex.c)
 */
void Bootloader_EraseFlash_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint8_t eraseStatus = 0x00;
	Print_Msg("BL_DEBUG_MSG: Bootloader_EraseFlash_Cmd_Handler()\n");
	
	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 1);
		Print_Msg("BL_DEBUG_MSG: Initial sector: %d, number of sectors: %d\n", pBLRxBuffer[2], pBLRxBuffer[3]);
		
		/* Configure an on-board LED to turn on when the erase operation begins,
		 * and turn off when the operation ends.
		 */
		HAL_GPIO_WritePin(LD4_GPIO_PORT, LD4_Pin, 1);
		eraseStatus = Execute_Flash_Erase(pBLRxBuffer[2], pBLRxBuffer[3]);
		HAL_GPIO_WritePin(LD4_GPIO_PORT, LD4_Pin, 0);

		Print_Msg("BL_DEBUG_MSG: Flash erase status: %#x\n", eraseStatus);
		
		/* Send response to the host */
		Bootloader_UART_Write_Data(&eraseStatus, 1);
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_EraseFlash_Cmd_Handler */

/**
 * Bootloader_ReadMem_Cmd_Handler()
 * Brief	: Handles BL_READ_MEM command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_READ_MEM command is used to read data from different memories of
 * 			  the MCU.
 */
void Bootloader_ReadMem_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	// TODO
} /* End of Bootloader_ReadMem_Cmd_Handler */

/**
 * Bootloader_WriteMem_Cmd_Handler()
 * Brief	: Handles BL_WRITE_MEM command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_WRITE_MEM command is used to write data into different memories
 *			  of the MCU.
 *			  BL_WRITE_MEM command can handle writing maximum 255 bytes at once.
 *			  The host application will use this command repeatedly to write all
 *			  required data to the specified memory.
 *			  The bootloader then will write the contents of the 'payload' field
 *			  of the packet frame to the memory specified by the 'base memory 
 *			  address' field.
 */
void Bootloader_WriteMem_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint8_t addrValid = ADDR_VALID;
	uint8_t writeStatus = 0x00;
	uint8_t checksum = 0, len = 0;
	len = pBLRxBuffer[0];
	uint8_t payloadLen = pBLRxBuffer[6];
	
	uint32_t memAddr = *((uint32_t *)&pBLRxBuffer[2]);
	
	checksum = pBLRxBuffer[len];
	
	Print_Msg("BL_DEBUG_MSG: Bootloader_WriteMem_Cmd_Handler()\n");

	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 1);
		
		Print_Msg("BL_DEBUG_MSG: Memory address to write to: %#x\n", memAddr);
		
		if (Verify_Addr(memAddr) == ADDR_VALID)
		{
			Print_Msg("BL_DEBUG_MSG: Valid address for write\n");
			
			/* Turn on the LED to indicate that the bootloader's write operation is on */
			HAL_GPIO_WritePin(LD4_GPIO_PORT, LD4_PIN, GPIO_PIN_SET);
			/* Execute memory write */
			writeStatus = Execute_Memory_Write(&pBLRxBuffer[7], memAddr, payloadLen);
			
			/* Turn off the LED to indicate that the bootloader's write operation is over */
			HAL_GPIO_WritePin(LD4_GPIO_PORT, LD4_PIN, GPIO_PIN_RESET);
			
			/* Send response to the host */
			Bootloader_UART_Write_Data(&writeStatus, 1);
		}
		else
		{
			Print_Msg("BL_DEBUG_MSG: Invalid address for memory write\n");
			writeStatus = ADDR_INVALID;
			
			/* Send response to the host */
			Bootloader_UART_Write_Data(&writeStatus, 1);
		}
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_WriteMem_Cmd_Handler */

/**
 * Bootloader_GetRDPLevel_Cmd_Handler()
 * Brief	: Handles BL_GET_RDP_LEVEL command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_GET_RDP_LEVEL command is used to read the Flash Read Protection
 *            (RDP) level.
 */
void Bootloader_GetRDPLevel_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint8_t rdpLevel = 0x00;
	Print_Msg("BL_DEBUG_MSG: Bootloader_GetRDPLevel_Cmd_Handler()\n");

	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 1);
		
		rdpLevel = Get_Flash_RDP_Level();
		Print_Msg("BL_DEBUG_MSG: RDP level: %d %#x\n", rdpLevel, rdpLevel);
		
		/* Send response to the host */
		Bootloader_UART_Write_Data(&rdpLevel, 1);
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_GetRDPLevel_Cmd_Handler */

/**
 * Bootloader_SetRDPLevel_Cmd_Handler()
 * Brief	: Handles BL_SET_RDP_LEVEL command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_SET_RDP_LEVEL command is used to set the Flash Read Protection
 *            (RDP) level.
 *			  - When Level 2 is activated, the Level of protection cannot be 
 *				degraded to Level 1 or Level 0. This is an irreversible operation.
 *			  - Switching to Level 2 should only be done during the production
 *				phase to impose some restrictions before handing the product to
 *				the customer so that they cannot access or modify the production
 *				code.
 *			  - During the development phase, DO NOT change the RDP status to Level 2!
 */
void Bootloader_SetRDPLevel_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint8_t rdpLevel;
	Print_Msg("BL_DEBUG_MSG: Bootloader_SetRDPLevel_Cmd_Handler()\n");

	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 1);
		
		rdpLevel = pBLRxBuffer[2];
		Set_Flash_RDP_Level(rdpLevel);
		Print_Msg("BL_DEBUG_MSG: RDP level: %d %#x\n", rdpLevel, rdpLevel);
		
		/* Send response to the host */
		Bootloader_UART_Write_Data(&rdpLevel, 1);
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_SetRDPLevel_Cmd_Handler */

/**
 * Bootloader_EnableWRP_Cmd_Handler()
 * Brief	: Handles BL_ENABLE_WRP command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_ENABLE_WRP command is used to enable the Write Protection (WRP)
 *			  for the selected sectors of the user Flash memory.
 */
void Bootloader_EnableWRP_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint8_t status = 0x00;
	
	Print_Msg("BL_DEBUG_MSG: Bootloader_EnableWRP_Cmd_Handler()\n");
	
	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 1);
		
		status = Configure_Flash_WRP(pBLRxBuffer[2], 0);
		
		Print_Msg("BL_DEBUG_MSG: Write protection enable status: %#x\n", status);
		
		/* Send response to the host */
		Bootloader_UART_Write_Data(&status, 1);
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_EnableWRP_Cmd_Handler */

/**
 * Bootloader_DisableWRP_Cmd_Handler()
 * Brief	: Handles BL_DISABLE_WRP command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_DISABLE_WRP command is used to disable the Write Protection (WRP)
 *			  for all the sectors of the user Flash (i.e., Restores the default
 *			  protection state)
 */
void Bootloader_DisableWRP_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint8_t status = 0x00;
	
	Print_Msg("BL_DEBUG_MSG: Bootloader_DisableWRP_Cmd_Handler()\n");
	
	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 1);
		
		status = Configure_Flash_WRP(0, 1);
		
		Print_Msg("BL_DEBUG_MSG: Write protection disable status: %#x\n", status);
		
		/* Send response to the host */
		Bootloader_UART_Write_Data(&status, 1);
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}
} /* End of Bootloader_DisableWRP_Cmd_Handler */

/**
 * Bootloader_GetWRPStatus_Cmd_Handler()
 * Brief	: Handles BL_GET_WRP_STATUS command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_GET_WRP_STATUS command is used to read the read/write 
 *			  protection status of all the sectors of the user Flash memory.
 */
void Bootloader_GetWRPStatus_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint16_t status;
	
	Print_Msg("BL_DEBUG_MSG: Bootloader_DisableRWProtect_Cmd_Handler()\n");
	
	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));

	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], cmdPacketLen - 4, crcHost))
	{
		/* Checksum is correct */
		Print_Msg("BL_DEBUG_MSG: Checksum verification success!\n");
		
		/* Send ACK */
		Bootloader_Tx_ACK(pBLRxBuffer[0], 2);
		
		//status = Read_OB_nWRP_RDP_Status();	/* TODO */
		
		Print_Msg("BL_DEBUG_MSG: nWRP: %#x, RDP: %#x\n", status >> 16, (status & 0xFFFF) >> 8);
		
		/* Send response to the host */
		//Bootloader_UART_Write_Data(&status, 1); /* TODO */
	}
	else
	{
		/* Checksum is not correct */
		Print_Msg("BL_DEBUG_MSG: Checksum vrification fail!\n");
		
		/* Send NACK */
		Bootloader_Tx_NACK();
	}

} /* End of Bootloader_ReadProtectStatus_Cmd_Handler */

/**
 * Bootloader_ReadOTP_Cmd_Handler()
 * Brief	: Handles BL_READ_OTP command
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 * Retval	: None
 * Note		: BL_READ_OTP command is used to read the OTP contents
 */
void Bootloader_ReadOTP_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	// TODO
} /* End of Bootloader_ReadOTP_Cmd_Handler */


/* Implementation of the bootloader helper functions -------------------------*/

/**
 * Bootloader_Tx_ACK()
 * Brief	: Sends ACK message to the host
 * Param	: @cmdCode - Command code received from the host
 *			: @lenToFollow - Length of the subsequent data bytes
 * Retval	: None
 */
void Bootloader_Tx_ACK(uint8_t cmdCode, uint8_t lenToFollow)
{
	/* ACKing incorporates 2 bytes; ACK , length to follow */
	uint8_t ackBuf[2];
	ackBuf[0] = BL_ACK;
	ackBuf[1] = lenToFollow;
	HAL_UART_Transmit(C_UART, ackBuf, 2, HAL_MAX_DELAY);
} /* End of Bootloader_Tx_ACK */

/**
 * Bootloader_Tx_NACK()
 * Brief	: Sends NACK message to the host
 * Param	: None
 * Retval	: None
 */
void Bootloader_Tx_NACK(void)
{
	/* NACKing incorporates 1 byte */
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
} /* End of Bootloader_Tx_NACK */

/**
 * Bootloader_Verify_CRC()
 * Brief	: Verifies the CRC of the given buffer in data
 * Param	: @pPacket - Pointer to the data for which the CRC has to be calculated
 *			: @len - Length for which the CRC has to be calculated
 *			: @crcHost - 32-bit crc value sent by the host
 * Retval	: 0 if CRC verification was successful, 1 otherwise.
 */
uint8_t Bootloader_Verify_CRC(uint8_t *pPacket, uint32_t len, uint32_t crcHost)
{
	uint32_t crc = 0xFF;
	
	for (uint32_t i = 0; i < len; i++)
	{
		uint32_t dataByte = pPacket[i];
		crc = HAL_CRC_Accumulate(&hcrc, &dataByte, 1);
	}
	/* Reset CRC Calculation unit.
	 * Without this, all CRC checks following the very first wone will fail.
	 */
	__HAL_CRC_DR_RESET(&hcrc);
		
	if (crc == crcHost)
	{
		return CRC_VERIFICATION_SUCCESS;
	}
	
	return CRC_VERIFICATION_FAIL;
} /* End of Bootloader_Verify_CRC */

/**
 * Bootloader_UART_Write_Data()
 * Brief	: Writes data to C_UART
 * Param	: @pBLRxBuffer - Pointer to the bootloader's Rx buffer
 *			: @len - Length of data to write
 * Retval	: None
 * Note		: This function is a wrapper function for 'HAL_UART_Transmit()'
 */
void Bootloader_UART_Write_Data(uint8_t *pBuffer, uint32_t len)
{
	HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
} /* End of Bootloader_UART_Write_Data */

/**
 * Get_Bootloader_Version()
 * Brief	: Returns the bootloader version macro value
 * Param	: None
 * Retval	: Bootloader version macro value
 */
uint8_t Get_Bootloader_Version(void)
{
	return (uint8_t)BL_VERSION;
} /* End of Get_Bootloader_Version */

/**
 * Get_MCU_Chip_ID()
 * Brief	: Returns the chip (or device) identifier
 * Param	: None
 * Retval	: 16-bit MCU chip identification number
 * Note		: For more information, see the "MCU device ID code" section of the
 *			  MCU reference manual. (DBGMCU_IDCODE; Address: 0xE0042000)
 *			  - Bit[31:16] - Revision identifier (REV_ID)
 *			  - Bit[11:0] - Device identifier (DEV_ID) - 0x413
 */
uint16_t Get_MCU_Chip_ID(void)
{
	return (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
} /* End of Get_MCU_Chip_ID */

/**
 * Get_Flash_RDP_Level()
 * Brief	: Returns the Flash memory RDP (Read Protection) level
 * Param	: None
 * Retval	: 8-bit RDP level
 * Note		: For more information, see the "Option Bytes" section of the MCU
 *			  reference manual.
 *			  Additional functions can be defined to handle RDP level change.
 */
uint8_t Get_Flash_RDP_Level(void)
{
	uint8_t rdpStatus = 0;
#if 0
	/* The following code shows how to use the APIs provided by the Flash driver
	 * of the ST hardware abstraction layer to achieve reading the RDP status.
	 */
	FLASH_OBProgramInitTypeDef obHandle;
	HAL_FLASHEx_OBGetConfig(&obHandle);
	rdpStatus = (uint8_t)obHandle.RDPLevel;
#else
	volatile uint32_t *pOBAddr = (uint32_t *)0x1FFFC000; /* Option byte addr */
	rdpStatus = (uint8_t)(*pOBAddr >> 8);	/* Extract bits[15:8] */
#endif
	
	return rdpStatus;
} /* End of Get_Flash_RDP_Level */

/**
 * Set_Flash_RDP_Level()
 * Brief	: Modifies the user option bytes
 * Param	: @rdpLevel - Read Protection (RDP) level; 0, 1, or 2
 *			  to the default status otherwise
 * Retval	: 8-bit satus value; 0
 * Note		: To modify the user option value, follow the following sequence:
 *			  1. Check that no Flash memory operation is ongoing by checking the
 *				 BSY bit in the FLASH_SR register
 *			  2. Write the desired option value in the FLASH_OPTCR register
 *			  3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
 *			  4. Wait for the BSY bit to be cleared
 *			  Please see the reference manual for more details.
 *			  This function is used by both 'Bootloader_SetRDPLevel_Cmd_Handler()'.
 */
uint8_t Set_Flash_RDP_Level(uint8_t rdpLevel)
{
	/* Configure read protection level according to @rdpLevel ----------------*/
		 
	/* Option byte configuration unlock */
	HAL_FLASH_OB_Unlock();
	
	/* Wait till there's no active operation on Flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);		
		
	/* Clear bits[8:15] of FLASH_OPTCR before writing */
	FLASH_OPTCR &= ~(0xFF << 8);

	if (rdpLevel == 0)
	{
		/* Read protection level 0: Configure bits[8:15] of FLASH_OPTCR to 0xAA */
		FLASH_OPTCR |= (RDP_LEVEL0 << 8);
	}
	else if (rdpLevel == 1)
	{
		/* Read protection level 1: Configure bits[8:15] of FLASH_OPTCR to 0x00 */
		FLASH_OPTCR |= (RDP_LEVEL1 << 8);
	}
	else
	{
		/* Read protection level 1: Configure bits[8:15] of FLASH_OPTCR to 0xCC */
		FLASH_OPTCR |= (RDP_LEVEL2 << 8);
	}
	
	/* Set the option start bit (OPTSTRT) in the FLASH_OPTCR register */
	FLASH_OPTCR |= (0x1 << 1);
	
	/* Wait till there's no active operation on Flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	
	HAL_FLASH_OB_Lock();
	
	return 0;
} /* End of Set_Flash_RDP_Level */

/**
 * Verify_Addr()
 * Brief	: Verifies the address sent by the host
 * Param	: @addr - Address sent by the host that is to be verified
 * Retval	: ADDR_VALID if the passed addr is valid, ADDR_INVALID otherwise
 * Note		: What are the valid address to which the code can jump?
 *			  - System memory: YES
 *			  - SRAM1: YES
 *			  - SRAM2: YES
 *			  - Backup SRAM: YES
 *			  - Peripheral memory: NO (Possible, but will not allow)
 *			  - External memory:	YES
 */
uint8_t Verify_Addr(uint32_t addr)
{
	if (SRAM1_BASE <= addr && addr <= SRAM1_END)
		return ADDR_VALID;
	else if (SRAM2_BASE <= addr && addr <= SRAM2_END)
		return ADDR_VALID;
	else if (FLASH_BASE <= addr && addr <= FLASH_END)
		return ADDR_VALID;
	else if (BKPSRAM_BASE <= addr && addr <= BKPSRAM_END)
		return ADDR_VALID;
	else
		return ADDR_INVALID;
} /* End of Verify_Addr */

/**
 * Execute_Flash_Erase()
 * Brief	: Performs Flash erase as per the passed arguments
 * Param	: @sectorNumber - The sector number (0-11) to erase or 0xFF to mass-
 *			  erase
 *			  @numberOfSectors - The number of sectors to erase 
 * Retval	: 0x00 (HAL_OK) if the erase was successful
 *			  0x01 (HAL_ERROR) if the erase was unsuccessful due to error
 *			  0x02 (HAL_BUSY) if the erase was unsuccessful due to busy state
 *			  0x03 (HAL_TIMEOUT) if the erase was unsuccessful due to timeout
 *			  0x04 (INVALID_NUM_OF_SECTORS) if the entered number of sectors to
 *				   erase was invalid
 * Note		: Main memory of the Flash module of the STM32F407xx MCUs consists
 *			  of 12 sectors (0-11). Therefore, @numberOfSectors should be in the
 *			  range of 1-12.
 *			  @sectorNumber 0xFF means 'mass-erase'.
 */
uint8_t Execute_Flash_Erase(uint8_t sectorNumber, uint8_t numberOfSectors)
{
	FLASH_EraseInitTypeDef flashEraseHandle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;
	
	if (numberOfSectors > MAX_NUM_OF_SECTORS)
		return INVALID_NUM_OF_SECTORS;
	
	if ((sectorNumber == 0xFF) || (sectorNumber < MAX_NUM_OF_SECTORS))
	{
		if (sectorNumber == (uint8_t)0xFF)
		{
			/* Perform mass-erase */
			flashEraseHandle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			/* Perform sector-erase */
			
			/* Calculate how many sectors need to be erased */
			uint8_t remainingSectors = MAX_NUM_OF_SECTORS - sectorNumber;
			if (numberOfSectors > remainingSectors)
			{
				numberOfSectors = remainingSectors;
			}
			
			/* Populate the flashEraseHandle structure */
			flashEraseHandle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashEraseHandle.Sector = sectorNumber;	/* Initial sector */
			flashEraseHandle.NbSectors = numberOfSectors;
		}
		
		flashEraseHandle.Banks = FLASH_BANK_1;
		
		/* In order to access the Flash registers, unlock the flash first.
		 * ('Unlocking the Flash control register' of the MCU reference manual) 
		 */
		HAL_FLASH_Unlock();
		flashEraseHandle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		status = (uint8_t)HAL_FLASHEx_Erase(&flashEraseHandle, &sectorError);
		HAL_FLASH_Lock();
		
		return status;
	}
	
	return INVALID_NUM_OF_SECTORS;
} /* End of Execute_Flash_Erase */

/**
 * Execute_Memory_Write()
 * Brief	: Writes the contents of @pBuffer byte-by-byte to @memAddr
 * Param	: @pBuffer - Pointer to a buffer that contains data to write to 
 *			  memory
 *			  @memAddr - Memory address to write to 
 *			  @len - byte-length of the data to write
 * Retval	: HAL_OK		= 0x00U,
 *			  HAL_ERROR		= 0x01U,
 * 			  HAL_BUSY		= 0x02U,
 *			  HAL_TIMEOUT  	= 0x03U
 * Note		: Current implementation supports writing to Flash only.
 *			  This function does not check whether 'memAddr' is valid address of
 *			  the Flash range.
 *			  The API 'HAL_FLASH_Program' performs the Flash memory programming
 *			  according to the 'Flash memory programming sequence' described in
 *			  the 'Programming - Standard programming' section of the MCU
 *			  reference manual.
*/
uint8_t Execute_Memory_Write(uint8_t *pBuffer, uint32_t memAddr, uint32_t len)
{
	uint8_t status = HAL_OK;
	
	/* In order to access the Flash registers, unlock the flash first.
	 * ('Unlocking the Flash control register' of the MCU reference manual) 
	 */
	HAL_FLASH_Unlock();/* USER CODE END 4 */
	
	for (uint32_t i = 0; i < len; i++)
	{
		/* Program the Flash byte-by-byte */
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, memAddr + i, pBuffer[i]);
	}
	
	HAL_FLASH_Lock();
	
	return status;
} /* End of Execute_Memory_Write */

/**
 * Configure_Flash_WRP()
 * Brief	: Modifies the user option bytes
 * Param	: @nwrp - Sector not write protect value
 *			  @disable - Allows configuring WRP status when 0, resets WRP back
 *			  to the default status otherwise
 * Retval	: 8-bit satus value; 0
 * Note		: To modify the user option value, follow the following sequence:
 *			  1. Check that no Flash memory operation is ongoing by checking the
 *				 BSY bit in the FLASH_SR register
 *			  2. Write the desired option value in the FLASH_OPTCR register
 *			  3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
 *			  4. Wait for the BSY bit to be cleared
 *			  Please see the reference manual for more details.
 *			  This function is used by both 
 *			  'Bootloader_EnableWRP_Cmd_Handler()', and
 *			  'Bootloader_DisableWRP_Cmd_Handler()'
 */
uint8_t Configure_Flash_WRP(uint8_t nwrp, uint8_t disable)
{
	if (disable)
	{
		/* Disable read/write protection on all the sectors of the user Flash 
		 * (i.e., Restores the default protection state)
		 */
		
		/* Option byte configuration unlock */
		HAL_FLASH_OB_Unlock();
		
		/* Wait till there's no active operation on Flash */
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		
		/* Clear the protection; set all sector bits[27:16] (nWRPi) to 1 */
		FLASH_OPTCR |= (0xFFF << 16);
		
		/* Set the option start bit (OPTSTRT) in the FLASH_OPTCR register */
		FLASH_OPTCR |= (0x1 << 1);
		
		/* Wait till there's no active operation on Flash */
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		
		HAL_FLASH_OB_Lock();
		
		return 0;
	}

	/* Configure write protection on the sectors encoded in @nwrp ------------*/

	/* Option byte configuration unlock */
	HAL_FLASH_OB_Unlock();
	
	/* Wait till there's no active operation on Flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);		
	
	/* Set write protection on the sectors encoded in @sectorDetails */
	FLASH_OPTCR &= ~(nwrp << 16);
		
	/* Set the option start bit (OPTSTRT) in the FLASH_OPTCR register */
	FLASH_OPTCR |= (0x1 << 1);
	
	/* Wait till there's no active operation on Flash */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	
	HAL_FLASH_OB_Lock();
	
	return 0;
} /* End of Configure_Flash_WRP */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
