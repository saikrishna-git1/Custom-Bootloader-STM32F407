/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if 1
#define BL_DEBUG_MSG_EN
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

#define CMD_UART   &huart2
#define DBG_UART   &huart3

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void printmsg(char *format,...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char someData[] = "Hello from Bootloader\n";

#define BL_RX_LEN 200
uint8_t bl_rx_buff[BL_RX_LEN];

#if 0
	const uint8_t array[0x8000] = {0};
#endif
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

#if 0
  int somevar = 0;
  for(int i = 0; i < 0x8000; i++) {
	  somevar = array[i];
  }
#endif


  /* USER CODE BEGIN 2 */
  if( HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) == GPIO_PIN_SET )
    {
  	  printmsg("[BL_DBG]: Button is pressed, going to BL mode...\n");
  	  //if User button is pressed, bootloader should wait for command
  	  bootloader_uart_read_data();
    }
    else
    {
  	  printmsg("[BL_DBG]: Button is not pressed, will jump to user_application.\n");
  	  //if User button is not pressed, jump to user application
  	  bootloader_jump_to_user_app();
    }
  /* USER CODE END 2 */


#if 0
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //HAL_UART_Transmit(&huart2, (uint8_t *) someData, sizeof(someData), HAL_MAX_DELAY); //testing 'Command UART'
	  //HAL_UART_Transmit(&huart3, (uint8_t *) someData, sizeof(someData), HAL_MAX_DELAY); //testing 'Debug UART'
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
#endif
} //main()



/***************** Implementation of common functions **************************/

uint8_t bl_verify_crc (uint8_t *pData, uint32_t len, uint32_t host_crc) {
	uint32_t bl_crc=0xFF;

    for (uint32_t i=0 ; i < len ; i++) {
        uint32_t i_data = pData[i];
        bl_crc = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

  __HAL_CRC_DR_RESET(&hcrc); //soft reset CRC module

	if( bl_crc == host_crc) {
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}


void bl_send_ack(uint8_t len_to_follow) {
	//we send 2 bytes here - ACK, len_to_follow in bytes
	uint8_t ack_buff[2] = {0};
	ack_buff[0] = BL_ACK;
	ack_buff[1] = len_to_follow;
	HAL_UART_Transmit(CMD_UART, ack_buff, 2, HAL_MAX_DELAY);
}


void bl_send_nack(void) {
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(CMD_UART, &nack, 1, HAL_MAX_DELAY);
}


void bl_uart_write_data(uint8_t *pData, uint8_t len) {
	HAL_UART_Transmit(CMD_UART, pData, len, HAL_MAX_DELAY);
}


uint8_t mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len) {
    uint8_t status=HAL_OK;

    //We have to unlock flash module to get control of registers
    HAL_FLASH_Unlock();

    for(uint32_t i = 0 ; i <len ; i++)
    {
        //Here we program the flash byte by byte
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,pBuffer[i] );
    }

    HAL_FLASH_Lock();

    return status;
}



//verify the address sent by the host .
uint8_t verify_address(uint32_t go_address) {
	return ADDR_VALID;
}


/*******************************************************************************/




/****************** Implementation of BL CMD Handle functions ******************/

void bl_get_ver_handle(uint8_t *bl_rx_buff) {
	printmsg("[BL_DBG]: In %s\n", __func__);

	uint8_t bl_ver = 0;

	//total length of the cmd_pkt = length_to_follow + cmd + CRC
	uint8_t cmd_pkt_len = 6;

	//extract host_crc
	uint32_t host_crc = *( (uint32_t *) &bl_rx_buff[2] );
	//uint32_t host_crc = *( (uint32_t *) (bl_rx_buff + cmd_pkt_len - 4) );

	if( !(bl_verify_crc(bl_rx_buff, cmd_pkt_len - 4, host_crc)) ) {
		//CRC pass
		printmsg("[BL_DBG]: CRC PASS !!\n");
		bl_send_ack(1); //only 1 Byte will be send in reply
		bl_ver = (uint8_t) BL_VERSION;
		printmsg("[BL_DBG]: BL_VERSION = %d, 0x%x\n", bl_ver, bl_ver);
		bl_uart_write_data(&bl_ver, 1);
	}
	else {
		//CRC fail
		printmsg("[BL_DBG]: CRC FAIL !!!!!\n");
		bl_send_nack();
	}

}


void bl_get_help_handle(uint8_t *bl_rx_buff) {

}


void bl_get_cid_handle(uint8_t *bl_rx_buff) {

}




/*******************************************************************************/


void bl_mem_write_cmd_handle(uint8_t *pBuffer)
{
	//uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	//uint8_t len=0;
	//len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );

	//chksum = pBuffer[len];

    printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buff[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buff+command_packet_len - 4) ) ;


	if (! bl_verify_crc(&bl_rx_buff[0],command_packet_len-4,host_crc))
	{
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        bl_send_ack(1);

        printmsg("BL_DEBUG_MSG: mem write address : %#x\n",mem_address);

		if( verify_address(mem_address) == ADDR_VALID )
		{

            printmsg("BL_DEBUG_MSG: valid mem write address\n");

            //glow the led to indicate bootloader is currently writing to memory
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

            //execute mem write
            write_status = mem_write(&pBuffer[7],mem_address, payload_len);

            //turn off the led to indicate memory write is over
            HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

            //inform host about the status
            bl_uart_write_data(&write_status,1);

		}else
		{
            printmsg("BL_DEBUG_MSG: invalid mem write address\n");
            write_status = ADDR_INVALID;
            //inform host that address is invalid
            bl_uart_write_data(&write_status,1);
		}


	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bl_send_nack();
	}

}



void bootloader_uart_read_data(void) {
	uint8_t pkt_len_to_follow = 0;

	while(1) {
		memset(bl_rx_buff, 0, 200);
		//here, read and decode commands from the Host
		//first, read only the 'length' field of the CMD packet and then read 'length' bytes
		HAL_UART_Receive(CMD_UART, bl_rx_buff, 1, HAL_MAX_DELAY);
		pkt_len_to_follow = bl_rx_buff[0];
		HAL_UART_Receive(CMD_UART, &bl_rx_buff[1], pkt_len_to_follow, HAL_MAX_DELAY);

		//based on the CMD, process accordingly
		switch(bl_rx_buff[1]) {
			case BL_GET_VER:
				bl_get_ver_handle(bl_rx_buff);
				break;
			case BL_GET_HELP:
				bl_get_help_handle(bl_rx_buff);
				break;
			case BL_GET_CID:
				bl_get_cid_handle(bl_rx_buff);
				break;
			case BL_MEM_WRITE:
				bl_mem_write_cmd_handle(bl_rx_buff);
			default:
				printmsg("[BL_DBG]: Invalid bootloader command\n");
				break;
		}
	}
}



void bootloader_jump_to_user_app(void) {
	//fn_ptr to hold the address of the user_application's reset_handler
	void (*fn_ptr_void_void) (void) = NULL;

#if 0 //is this really needed ?? Working without this. Taken care by the reset_handler of user_application
	//configure MSP by reading the first word of the user_application region in Flash memory
	uint32_t msp_value = *( (volatile uint32_t *) FLASH_SECTOR2_BASE_ADDRESS );
	printmsg("[BL_DBG]: MSP value = 0x%x\n", msp_value);
	__set_MSP(msp_value); //CMSIS API
#endif

	//store the addr of user_application's reset_handler in the fn_ptr
	uint32_t reset_handler_addr = *( (volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4) );
	printmsg("[BL_DBG]: reset_handler_addr of user_application = 0x%x\n", reset_handler_addr);
	fn_ptr_void_void = (void *) reset_handler_addr;

	//jump to the reset_handler of user_application
	printmsg("[BL_DBG]: bootloader jumping to user_application...\n");
	fn_ptr_void_void();
} //void bootloader_jump_to_user_app(void)


/* prints formatted string to console over UART */
 void printmsg(char *format,...)
 {
#ifdef BL_DEBUG_MSG_EN
	char str[80];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	HAL_UART_Transmit(DBG_UART,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
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
