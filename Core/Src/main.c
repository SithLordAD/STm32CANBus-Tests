/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

uint32_t              TxMailbox;       /* The number of the mail box that transmitted the Tx message */
CAN_TxHeaderTypeDef   TxHeader;        /* Header containing the information of the transmitted frame */
uint8_t               TxData[8] = {0}; /* Buffer of the data to send */
CAN_RxHeaderTypeDef   RxHeaderFIFO0;   /* Header containing the information of the received frame */
uint8_t               RxDataFIFO0[8];  /* Buffer of the received data */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);


int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();

    TxHeader.StdId = 0x200;              /* The ID value in Standard ID format (11bit) */
    TxHeader.RTR = CAN_RTR_DATA;         /* The frames that will be sent are Data */
    TxHeader.DLC = 8;                    /* The frames will contain 8 data bytes */
    TxHeader.TransmitGlobalTime = DISABLE;
    /* Only the first byte (data0) and the last byte (data7) will be changed */
    TxData[0] = 0;                       /* The first value to send on byte 0 is 0 */
    TxData[7] = 0xFF;                    /* The last value to send on byte 0 is 0xFF */

  while (1)
  {
    /* USER CODE END WHILE */
	TxData[0] ++; /* Increment the first byte */
	TxData[7] --; /* Increment the last byte */
	/* It's mandatory to look for a free Tx mail box */
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {} /* Wait till a Tx mailbox is free. Using while loop instead of HAL_Delay() */

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) /* Send the CAN frame */
	{
	/* Transmission request Error */
	Error_Handler();
	}

	/* Toggle sending Standard and Extended ID for the next operation. The first ID sent is a Standard ID frame */
	HAL_Delay(100);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_CAN1_Init(void)
{
	CAN_FilterTypeDef  sFilterConfig;

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 6;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	__HAL_RCC_CAN1_CLK_ENABLE();

	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}

	sFilterConfig.SlaveStartFilterBank = 14;           /* Slave start bank Set only once. */

	sFilterConfig.FilterBank = 0;                      /* Select the filter number 0 */
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* Using ID mask mode .. */
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* .. in 32-bit scale */
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;                /* The filter is set to receive only the Standard ID frames */
	sFilterConfig.FilterMaskIdHigh = 0x0000;           /* Accept all the IDs .. except the Extended frames */
	sFilterConfig.FilterMaskIdLow = 0x0000;            /* The filter is set to check only on the ID format */
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; /* All the messages accepted by this filter will be received on FIFO1 */
	sFilterConfig.FilterActivation = ENABLE;           /* Enable the filter number 0 */
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
   {
       /* Filter configuration Error */
       Error_Handler();
   }

   if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
   {
       /* Filter configuration Error */
       Error_Handler();
   }
   /* Start the CAN peripheral */
   if (HAL_CAN_Start(&hcan1) != HAL_OK)
   {
       /* Start Error */
       Error_Handler();
   }

   if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
     {
         /* Notification Error */
         Error_Handler();
     }
}

static void MX_USART2_UART_Init(void)
{

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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    /* Get RX message from FIFO0 and fill the data on the related FIFO0 user declared header
       (RxHeaderFIFO0) and table (RxDataFIFO0) */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeaderFIFO0, RxDataFIFO0) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }else{
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }

}

void Error_Handler(void)
{
  __disable_irq();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  while(1);
}




#ifdef USE_FULL_ASSERT
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
