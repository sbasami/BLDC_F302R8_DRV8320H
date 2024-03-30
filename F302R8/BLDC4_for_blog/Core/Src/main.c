/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLDC_PWM_FREQ 900 	// PWM周波数 900 = 40kHz
#define BLDC_PWM_ON 150		// ON  duty 150 / 900 * 100[%]
#define BLDC_PWM_OFF 0		// OFF duty
#define BLDC_PWM_ZEROCROSS_CHECK_TIMING 10 // ゼロクロス判定タイミングduty

// UVWのコンパレータ状態
#define COMPARATOR_STATE_0 0b101	// U:1  V:0  W:1
#define COMPARATOR_STATE_1 0b100	// U:1  V:0  W:0
#define COMPARATOR_STATE_2 0b110	// U:1  V:1  W:0
#define COMPARATOR_STATE_3 0b010	// U:0  V:1  W:0
#define COMPARATOR_STATE_4 0b011	// U:0  V:1  W:1
#define COMPARATOR_STATE_5 0b001	// U:0  V:0  W:1

// 転流状態
typedef enum
{
	COMMUTATION_STATE_0 = 0,
	COMMUTATION_STATE_1,
	COMMUTATION_STATE_2,
	COMMUTATION_STATE_3,
	COMMUTATION_STATE_4,
	COMMUTATION_STATE_5,
	COMMUTATION_STATE_ERROR,

}BLDC_CommutationState_enum;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;
COMP_HandleTypeDef hcomp6;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct
{
	int duty;
	int comm_state;					// 現在の転流状態
	int comm_flag;					// 転流したフラグ
	int zero_cross_interval_us;		// ゼロクロス間隔[us]
	int comp_UVW;					// 現在のコンパレータの状態(UVWまとめたやつ)
	int comp_UVW_old;				// 1つ前のコンパレータの状態
}BLDC_Data_struct;
BLDC_Data_struct BLDC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP4_Init(void);
static void MX_COMP6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void BLDC_Initialization(); 						// 初期化
void BLDC_Update_CommutationState(int state); 		// 実際に転流させる
void BLDC_Set_Duty(int duty); 						// Duty設定用
int BLDC_Get_Comparator(); 							// コンパレータの状態を取得
int BLDC_Get_NextCommutationState(int comp_uvw); 	// コンパレータの状態を渡して次の転流状態を取得
int BLDC_Get_ZeroCrossInterval(); 					// ゼロクロス間隔を取得
void BLDC_Set_NextCommutationTiming(int zero_cross_interval_us); // 次の転流タイミングを設定
void BLDC_Check_ZeroCross(); 						// ゼロクロス判定
void BLDC_TIM6_interrupt(); 						// TIM6割り込みコールバック
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BLDC_Initialization()
{
	// PWM duty
	BLDC_Set_Duty(BLDC_PWM_ON);
	// DRV8320H enable
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
	// Initialize duty value
	__HAL_TIM_SET_AUTORELOAD(&htim1, BLDC_PWM_FREQ-1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_PWM_OFF);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_PWM_OFF);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_PWM_OFF);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, BLDC_PWM_ZEROCROSS_CHECK_TIMING);
	// PWM start
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
	// Comparator start
	HAL_COMP_Start(&hcomp2);
	HAL_COMP_Start(&hcomp4);
	HAL_COMP_Start(&hcomp6);
	//Zero cross interval counter start
	HAL_TIM_Base_Start(&htim2);
}

void BLDC_Update_CommutationState(int state)
{
	switch(state)
	{
		case COMMUTATION_STATE_0:
			// U -> V
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC.duty);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_PWM_OFF);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_PWM_OFF);
			HAL_GPIO_WritePin(INL_U_GPIO_Port, INL_U_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INL_V_GPIO_Port, INL_V_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INL_W_GPIO_Port, INL_W_Pin, GPIO_PIN_RESET);
			break;

		case COMMUTATION_STATE_1:
			// U -> W
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC.duty);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_PWM_OFF);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_PWM_OFF);
			HAL_GPIO_WritePin(INL_U_GPIO_Port, INL_U_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INL_V_GPIO_Port, INL_V_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INL_W_GPIO_Port, INL_W_Pin, GPIO_PIN_SET);
			break;

		case COMMUTATION_STATE_2:
			// V -> W
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_PWM_OFF);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC.duty);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_PWM_OFF);
			HAL_GPIO_WritePin(INL_U_GPIO_Port, INL_U_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INL_V_GPIO_Port, INL_V_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INL_W_GPIO_Port, INL_W_Pin, GPIO_PIN_SET);
			break;

		case COMMUTATION_STATE_3:
			// V -> U
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_PWM_OFF);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC.duty);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_PWM_OFF);
			HAL_GPIO_WritePin(INL_U_GPIO_Port, INL_U_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INL_V_GPIO_Port, INL_V_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INL_W_GPIO_Port, INL_W_Pin, GPIO_PIN_RESET);
			break;

		case COMMUTATION_STATE_4:
			// W -> U
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_PWM_OFF);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_PWM_OFF);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC.duty);
			HAL_GPIO_WritePin(INL_U_GPIO_Port, INL_U_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INL_V_GPIO_Port, INL_V_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INL_W_GPIO_Port, INL_W_Pin, GPIO_PIN_RESET);
			break;

		case COMMUTATION_STATE_5:
			// W -> V
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_PWM_OFF);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_PWM_OFF);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC.duty);
			HAL_GPIO_WritePin(INL_U_GPIO_Port, INL_U_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INL_V_GPIO_Port, INL_V_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INL_W_GPIO_Port, INL_W_Pin, GPIO_PIN_RESET);
			break;

		default:
			break;
	}
}

void BLDC_Set_Duty(int duty)
{
	BLDC.duty = duty;
}

int BLDC_Get_Comparator()
{
	int comp_u;
	int comp_v;
	int comp_w;
	int comp_uvw;

	comp_u = (READ_BIT(hcomp2.Instance->CSR, COMP_CSR_COMPxOUT)) >> COMP_CSR_COMPxOUT_Pos;
	comp_v = (READ_BIT(hcomp4.Instance->CSR, COMP_CSR_COMPxOUT)) >> COMP_CSR_COMPxOUT_Pos;
	comp_w = (READ_BIT(hcomp6.Instance->CSR, COMP_CSR_COMPxOUT)) >> COMP_CSR_COMPxOUT_Pos;
	comp_uvw = (comp_u << 2) | (comp_v << 1) | comp_w;

	return comp_uvw;
}

int BLDC_Get_NextCommutationState(int comp_uvw)
{
	int comm_next_state;

	switch(comp_uvw)
	{
		case COMPARATOR_STATE_0:
			comm_next_state = COMMUTATION_STATE_0;
			break;
		case COMPARATOR_STATE_1:
			comm_next_state = COMMUTATION_STATE_1;
			break;
		case COMPARATOR_STATE_2:
			comm_next_state = COMMUTATION_STATE_2;
			break;
		case COMPARATOR_STATE_3:
			comm_next_state = COMMUTATION_STATE_3;
			break;
		case COMPARATOR_STATE_4:
			comm_next_state = COMMUTATION_STATE_4;
			break;
		case COMPARATOR_STATE_5:
			comm_next_state = COMMUTATION_STATE_5;
			break;
		default:
			comm_next_state = COMMUTATION_STATE_ERROR;
			break;
	}

	return comm_next_state;
}

int BLDC_Get_ZeroCrossInterval()
{
	int zero_cross_interval_us;

	zero_cross_interval_us = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	return zero_cross_interval_us;
}

void BLDC_Set_NextCommutationTiming(int zero_cross_interval_us)
{
	__HAL_TIM_SET_AUTORELOAD(&htim6, (int)(zero_cross_interval_us / 2));
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	HAL_TIM_Base_Start_IT(&htim6);
}

void BLDC_Check_ZeroCross()
{
	if(BLDC.comp_UVW != BLDC.comp_UVW_old)
	{
		BLDC.zero_cross_interval_us = BLDC_Get_ZeroCrossInterval();
		BLDC_Set_NextCommutationTiming(BLDC.zero_cross_interval_us);

		BLDC.comp_UVW_old = BLDC.comp_UVW;

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void BLDC_CC_interrupt()
{
	if(BLDC.comm_flag == 0)
	{
		BLDC.comp_UVW = BLDC_Get_Comparator();
		BLDC_Check_ZeroCross();
	}
	else
	{
		BLDC.comm_flag = 0;
	}
}

void BLDC_TIM6_interrupt()
{
	BLDC.comm_state = BLDC_Get_NextCommutationState(BLDC.comp_UVW);
	BLDC_Update_CommutationState(BLDC.comm_state);
	BLDC.comm_flag = 1;

	HAL_TIM_Base_Stop_IT(&htim6);

	HAL_GPIO_TogglePin(DEBUG_A_GPIO_Port, DEBUG_A_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6){
    	BLDC_TIM6_interrupt();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int bldc_step = 0;
	int i = 0;
	int duty = BLDC_PWM_ON;
	int freq = BLDC_PWM_FREQ;
	uint8_t buff[1];
	HAL_StatusTypeDef ret;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_COMP2_Init();
  MX_COMP4_Init();
  MX_COMP6_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  BLDC_Initialization();

  // Forced commutation
  for(i = 0; i < 5; i ++)
  {
	BLDC_Update_CommutationState(bldc_step);
	bldc_step++;
	bldc_step %= 6;
	HAL_Delay(5);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	ret = HAL_UART_Receive(&huart2, buff, 1, 1000);
	// B1スイッチを押しながらシリアルで「1」「2」を送信すると周波数が変化
	// B1スイッチを押さずにシリアルで「1」「2」を送信するとDutyが変化
	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
	{
		if (ret == HAL_OK)
		{
			if(buff[0] == '1')
			{
				duty += 10;
			}
			else if(buff[0] == '2')
			{
				duty -= 10;
			}
		}
		BLDC_Set_Duty(duty);
	}
	else
	{
		if (ret == HAL_OK)
		{
			if(buff[0] == '1')
			{
				freq += 10;
			}
			else if(buff[0] == '2')
			{
				freq -= 10;
			}
		}
		__HAL_TIM_SET_AUTORELOAD(&htim1, freq-1);
	}
	if (ret == HAL_OK)
	{
		char str[100] = {0};
		sprintf(str,"%d\t%d\r\n",duty,freq);
		HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof(str), 1000);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
  hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp2.Init.Output = COMP_OUTPUT_NONE;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InvertingInput = COMP_INVERTINGINPUT_IO2;
  hcomp4.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp4.Init.Output = COMP_OUTPUT_NONE;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief COMP6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP6_Init(void)
{

  /* USER CODE BEGIN COMP6_Init 0 */

  /* USER CODE END COMP6_Init 0 */

  /* USER CODE BEGIN COMP6_Init 1 */

  /* USER CODE END COMP6_Init 1 */
  hcomp6.Instance = COMP6;
  hcomp6.Init.InvertingInput = COMP_INVERTINGINPUT_IO2;
  hcomp6.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp6.Init.Output = COMP_OUTPUT_NONE;
  hcomp6.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp6.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  hcomp6.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP6_Init 2 */

  /* USER CODE END COMP6_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 900-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INL_W_Pin|LD2_Pin|INL_V_Pin|ENABLE_Pin
                          |DEBUG_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INL_U_GPIO_Port, INL_U_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INL_W_Pin LD2_Pin INL_V_Pin ENABLE_Pin
                           DEBUG_A_Pin */
  GPIO_InitStruct.Pin = INL_W_Pin|LD2_Pin|INL_V_Pin|ENABLE_Pin
                          |DEBUG_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INL_U_Pin */
  GPIO_InitStruct.Pin = INL_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INL_U_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
