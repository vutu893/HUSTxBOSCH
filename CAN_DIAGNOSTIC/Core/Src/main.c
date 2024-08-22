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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart3_receive;
uint8_t data_uart3_receive[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00} ;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef CAN1_pHeaderTx;
CAN_RxHeaderTypeDef CAN1_pHeaderRx;
CAN_FilterTypeDef CAN1_sFilterConfig;
CAN_TxHeaderTypeDef CAN2_pHeaderTx;
CAN_RxHeaderTypeDef CAN2_pHeaderRx;
CAN_FilterTypeDef CAN2_sFilterConfig;
uint32_t CAN1_pTxMailbox;
uint32_t CAN2_pTxMailbox;

uint16_t NumBytesReq = 1;
uint8_t  REQ_BUFFER  [4096];
uint8_t  REQ_1BYTE_DATA;
uint8_t sizedata;
uint8_t flg_check_receive_data = 0;

uint8_t CAN1_DATA_TX[8];
uint8_t CAN1_DATA_RX[8];
uint8_t CAN2_DATA_TX[8];
uint8_t CAN2_DATA_RX[8];

uint16_t Num_Consecutive_Tester;
uint8_t  Flg_Consecutive = 0;

//flag check  Receive data
uint8_t flg_CheckCan1Rx = 0;
uint8_t flg_CheckCan2Rx = 0;

unsigned int TimeStamp;
// maximum characters send out via UART is 30
char bufsend[30]="XXX: D1 D2 D3 D4 D5 D6 D7 D8  ";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void MX_CAN1_Setup();
void MX_CAN2_Setup();
void USART3_SendString(uint8_t *ch);
void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame);
void SID_22_Practice();
void SID_2E_Practice();
void SID_27_Practice();
void delay(uint16_t delay);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Callback function when press button on GPIO PIN 1(A1)


//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(flg_CheckCan2Rx)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,&CAN2_pHeaderRx, CAN2_DATA_RX);
	}
	if(flg_CheckCan1Rx)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,&CAN1_pHeaderRx, CAN1_DATA_RX);
	}
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,&CAN2_pHeaderRx, CAN2_DATA_RX);
//	if(CAN1_pHeaderRx.StdId == 0x0A2)
//	{
//		flg_CheckCan1Rx = 1;
//	}
}

// Transmit data through Can protocol
//void transmitDataCan1()
//void transmitDataCan2();
void printRequest();
//void printResponse();
//

void receiveITWithVariableLengthData(uint8_t size)
{
	HAL_UART_Receive_IT(&huart3, data_uart3_receive, size);
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//	delay(1000);
//	PrintCANLog(0x0012, data_uart3);
//}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_CAN1_Setup();
  MX_CAN2_Setup();
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

  // Initialize data frame of CAN1
  CAN1_pHeaderTx.DLC = 8;
  CAN1_pHeaderTx.IDE = CAN_ID_STD;
  CAN1_pHeaderTx.RTR = CAN_RTR_DATA;
  CAN1_pHeaderTx.StdId = 0x712;
  CAN1_pHeaderTx.TransmitGlobalTime = ENABLE;

  // Initialize data frame of CAN2
  CAN2_pHeaderTx.DLC = 8;
  CAN2_pHeaderTx.IDE = CAN_ID_STD;
  CAN2_pHeaderTx.RTR = CAN_RTR_DATA;
  CAN2_pHeaderTx.StdId = 0x7A2;
  CAN2_pHeaderTx.TransmitGlobalTime = ENABLE;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Example Function to print can message via uart
  //PrintCANLog(CAN1_pHeader.StdId, &CAN1_DATA_TX)
  HAL_UART_Receive_IT(&huart3, data_uart3_receive, 8);
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_UART_Receive_IT(&huart3, data_uart3_receive, 8) == HAL_OK)
	  {
		  if(flg_check_receive_data)
		  	  {
		  		  if(data_uart3_receive[0] == 0x22)
		  		  {
		  		  	SID_22_Practice();
		  		  	printRequest();
		  		  	delay(1000);
		  		  }
		  	  }
	  }
	  flg_check_receive_data = 0;
  }

  memset(&REQ_BUFFER,0x00,4096);
  NumBytesReq = 0;

  /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN1_sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  CAN1_sFilterConfig.FilterBank = 18;
  CAN1_sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  CAN1_sFilterConfig.FilterIdHigh = 0x7A2 << 5;
  CAN1_sFilterConfig.FilterIdLow = 0;
  CAN1_sFilterConfig.FilterMaskIdHigh = 0x7A2 << 5;
  CAN1_sFilterConfig.FilterMaskIdLow = 0x0000;
  CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN1_sFilterConfig.SlaveStartFilterBank = 20;
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 1;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  	CAN2_sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  	CAN2_sFilterConfig.FilterBank = 10;
  	CAN2_sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  	CAN2_sFilterConfig.FilterIdHigh = 0x712 << 5;
  	CAN2_sFilterConfig.FilterIdLow = 0;
  	CAN2_sFilterConfig.FilterMaskIdHigh = 0x712 << 5;
  	CAN2_sFilterConfig.FilterMaskIdLow = 0x0000;
  	CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  	CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  	CAN2_sFilterConfig.SlaveStartFilterBank = 0;
  /* USER CODE END CAN2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC4 PC5 PC6
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void MX_CAN1_Setup()
{
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
	HAL_CAN_Start(&hcan1);
	// enable interrupt CAN1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void MX_CAN2_Setup()
{
	HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig);
	HAL_CAN_Start(&hcan2);
	// enable interrupt CAN2
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void USART3_SendString(uint8_t *ch)
{
   while(*ch!=0)
   {
      HAL_UART_Transmit(&huart3, ch, 1,HAL_MAX_DELAY);
      ch++;
   }
}
void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame)
{
	uint16_t loopIndx = 0;
	char bufID[3] = "   ";
	char bufDat[2] = "  ";
	char bufTime [8]="        ";

	sprintf(bufTime,"%d",TimeStamp);
	USART3_SendString((uint8_t*)bufTime);
	USART3_SendString((uint8_t*)" ");

	sprintf(bufID,"%03X",CANID);
	for(loopIndx = 0; loopIndx < 3; loopIndx ++)
	{
		bufsend[loopIndx]  = bufID[loopIndx];
	}
	bufsend[3] = ':';
	bufsend[4] = ' ';


	for(loopIndx = 0; loopIndx < 8; loopIndx ++ )
	{
		sprintf(bufDat,"%02X",CAN_Frame[loopIndx]);
		bufsend[loopIndx*3 + 5] = bufDat[0];
		bufsend[loopIndx*3 + 6] = bufDat[1];
		bufsend[loopIndx*3 + 7] = ' ';
	}
	bufsend[29] = '\n';
	USART3_SendString((unsigned char*)bufsend);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	REQ_BUFFER[NumBytesReq] = REQ_1BYTE_DATA;
//	NumBytesReq++;
	//REQ_BUFFER[7] = NumBytesReq;
//	NumBytesReq++;
//	if (NumBytesReq > 8)
//	{
//		NumBytesReq = 1;
//	}
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//	delay(100);
	flg_check_receive_data = 1;
}
void delay(uint16_t delay)
{
	HAL_Delay(delay);
}
// CAN1 transmit and receive data
//void transmitDataCan1()
//{
//
//}
//
//// CAN2 transmit and receive data
//void transmitDataCan2()
//{
//
//
//}
//
void SID_22_Practice()
{
	// Receive data from UART3 and process data to DATA_CAN1_TX
	CAN1_DATA_TX[0] = 0x03;
	CAN1_DATA_TX[1] = data_uart3_receive[0];
	CAN1_DATA_TX[2] = data_uart3_receive[1];
	CAN1_DATA_TX[3] = data_uart3_receive[2];
	for(uint8_t i = 4; i < 8; i++)
	{
		CAN1_DATA_TX[i] = 0x55;
	}

}
//void SID_2E_Practice()
//{
//	// Receive data from UART3 and process data to DATA_CAN1_TX
//}
//void SID_27_Practice()
//{
//	// Receive data from UART3 and process data to DATA_CAN1_TX
//}
//
//// Print log request and response:
// request: CAN1 receive from USART3 and store to CAN1_DATA_TX
// response: be generated in CAN2 and via CAN communication be stored to CAN1_DATA_RX
// use CAN because requirement: must have ID CAN2 in DATA response ?right?
void printRequest()
{
	USART3_SendString((uint8_t *) "TESTER: ");
	PrintCANLog(CAN1_pHeaderTx.StdId, CAN1_DATA_TX);
	USART3_SendString((uint8_t *) "\n");
}
void printResponse()
{
	USART3_SendString((uint8_t *) "Response: ");
	PrintCANLog(CAN1_pHeaderRX.Stdid, CAN1_DATA_RX);
	USART3_SendString((uint8_t *) "\n");

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
