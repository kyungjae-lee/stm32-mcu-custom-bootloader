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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

char data[] = "Message from bootloader\r\n";
uint8_t blRxBuffer[BL_RX_LEN];

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
	uint8_t rcv_len = 0;
	
	while (1)
	{
		memset(blRxBuffer, 0, 200);
		
		/* Read and decode the command coming from the host */
		
		/* First, read the first byte (i.e., the "Length to follow" field ) */
		HAL_UART_Receive(C_UART, blRxBuffer, 1, HAL_MAX_DELAY);
		rcv_len = blRxBuffer[0];	/* Number of the remaining bytes to read */
		
		/* Second, read the remaining bytes, first of which is the "Command code" */
		HAL_UART_Receive(C_UART, &blRxBuffer[1], rcv_len, HAL_MAX_DELAY);
		
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
			case BL_GET_RDP_STATUS:
				Bootloader_GetRDP_Cmd_Handler(blRxBuffer);
				break;
			case BL_GO_TO_ADDR:
				Bootloader_GoToAddr_Cmd_Handler(blRxBuffer);
				break;
			case BL_ERASE_FLASH:
				Bootloader_EraseFlash_Cmd_Handler(blRxBuffer);
				break;
			case BL_WRITE_MEM:
				Bootloader_WriteMem_Cmd_Handler(blRxBuffer);
				break;
			case BL_ENDIS_RW_PROTECT:
				Bootloader_EnDisRWProtect_Cmd_Handler(blRxBuffer);
				break;
			case BL_READ_MEM:
				Bootloader_ReadMem_Cmd_Handler(blRxBuffer);
				break;
			case BL_READ_PROTECT_STATUS:
				Bootloader_ReadProtectStatus_Cmd_Handler(blRxBuffer);
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

/* Implementation of the bootloader command handler functions */

/* Helper function to handle BL_GET_VER command */
void Bootloader_GetVer_Cmd_Handler(uint8_t *pBLRxBuffer)
{
	uint8_t blVersion;
	
	/* 1. Verify the checksum value */
	Print_Msg("BL_DEBUG_MSG: bootloader_getver_cmd_handler()\n");
	
	/* Total length of the command packet */
	uint32_t cmdPacketLen = pBLRxBuffer[0] + 1;
	
	/* Extract the CRC32 sent by the host */
	uint32_t crcHost = *((uint32_t *)(pBLRxBuffer + cmdPacketLen - 4));
	
	if (!Bootloader_Verify_CRC(&pBLRxBuffer[0], pBLRxBuffer[0] - 4, crcHost))
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
}

/* Helper function to handle BL_GET_HELP command */
void Bootloader_GetHelp_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_GET_CID command */
void Bootloader_GetCID_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_GET_RDP command */
void Bootloader_GetRDP_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_GO_TO_ADDR command */
void Bootloader_GoToAddr_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_ERASE_FLASH command */
void Bootloader_EraseFlash_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_WRITE_MEM command */
void Bootloader_WriteMem_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_ENDIS_RW_PROTECT command */
void Bootloader_EnDisRWProtect_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_READ_MEM command */
void Bootloader_ReadMem_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_READ_PROTECT_STATUS command */
void Bootloader_ReadProtectStatus_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}

/* Helper function to handle BL_READ_OTP command */
void Bootloader_ReadOTP_Cmd_Handler(uint8_t *bl_rx_buffer)
{
}


/* Implementation of the bootloader helper functions */

void Bootloader_Tx_ACK(uint8_t cmdCode, uint8_t lenToFollow)
{
	/* ACKing incorporates 2 bytes; ACK , length to follow */
	uint8_t ackBuf[2];
	ackBuf[0] = BL_ACK;
	ackBuf[1] = lenToFollow;
	HAL_UART_Transmit(C_UART, ackBuf, 2, HAL_MAX_DELAY);
}

void Bootloader_Tx_NACK(void)
{
	/* NACKing incorporates 1 byte */
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}

/**
 * @brief	Verifies the CRC of the given buffer in data 
 * @param	data - Pointer to the data for which the CRC has to be calculated
 * @param	len - Length for which the CRC has to be calculated
 * @param	crc_host - 32-bit crc value sent by the host
 * @retval	0 if CRC verification was successful, 1 otherwise. 
 */
uint8_t Bootloader_Verify_CRC(uint8_t *pPacket, uint32_t len, uint32_t crcHost)
{
	uint32_t crc = 0xFF;
	
	for (uint32_t i = 0; i < len; i++)
	{
		uint32_t dataByte = pPacket[i];
		crc = HAL_CRC_Accumulate(&hcrc, &dataByte, 1);
		
		if (crc == crcHost)
		{
			return CRC_VERIFICATION_SUCCESS;
		}
	}
	return CRC_VERIFICATION_FAIL;
}

/**
 * @brief	Writes data to C_UART
 * @param	None
 * @retval	Bootloader version macro value
 * @note	This function is a wrapper function for 'HAL_UART_Transmit()'
 */
void Bootloader_UART_Write_Data(uint8_t *pBuffer, uint32_t len)
{
	HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
}

/**
 * @brief	Returns the bootloader version macro value
 * @param	None
 * @retval	Bootloader version macro value
 */
uint8_t Get_Bootloader_Version(void)
{
	return (uint8_t)BL_VERSION;
}

/* USER CODE END 4 */

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
