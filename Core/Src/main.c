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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// #define SENSOR_IN_Pin GPIO_PIN_3
// #define SENSOR_IN_GPIO_Port GPIOA
// #define BUTTON_0_Pin GPIO_PIN_4
// #define BUTTON_0_GPIO_Port GPIOA
// #define COMM_IN_Pin GPIO_PIN_5
// #define COMM_IN_GPIO_Port GPIOA
// #define MOTOR_2_A_Pin GPIO_PIN_6
// #define MOTOR_2_A_GPIO_Port GPIOA
// #define MOTOR_2_B_Pin GPIO_PIN_7
// #define MOTOR_2_B_GPIO_Port GPIOA
// #define MOTOR_1_A_Pin GPIO_PIN_8
// #define MOTOR_1_A_GPIO_Port GPIOA
// #define COMM_OUT_Pin GPIO_PIN_11
// #define COMM_OUT_GPIO_Port GPIOA
// #define MOTOR_1_B_Pin GPIO_PIN_12
// #define MOTOR_1_B_GPIO_Port GPIOA
// #define LED_0_Pin GPIO_PIN_5
// #define LED_0_GPIO_Port GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



#define SENSOR_COUNTER_MAX        2  /* defines step 1 = 2mm; 2 = 4mm ... */
volatile uint8_t sensor_counter = SENSOR_COUNTER_MAX;


#define STATE_IDLE                    1
#define STATE_MOVE_REEL_FORWARD       2
#define STATE_MOVE_REEL_BACKWARD      3
volatile uint8_t state_machine_status = STATE_IDLE;


#define BUTTON_STATUS_MASK_0_SENSOR         0
#define BUTTON_STATUS_MASK_1_BUTTON_0       1
// #define BUTTON_STATUS_MASK_1_BUTTON_0_LONG  2
#define BUTTON_STATUS_MASK_2_COMM_IN        3
// volatile uint8_t buttons_status = 0;
volatile uint8_t buttons_status = 1 << BUTTON_STATUS_MASK_1_BUTTON_0; /* set defaut value to 1 if you want "pressup" */
// volatile uint8_t buttons_status_long = 1 << BUTTON_STATUS_MASK_1_BUTTON_0;
volatile uint8_t buttons_status_long = 0;

volatile uint8_t buttons_status_flags = 0;
volatile uint8_t buttons_status_flags_long = 0;
volatile uint16_t button_debounce_counter_0 = 0;
volatile uint16_t button_debounce_counter_1 = 0;
volatile uint16_t button_debounce_counter_1_long = 0;
volatile uint16_t button_debounce_counter_2 = 0;

#define SENSOR_DEBOUNCE_VALUE  10*1 /* x100 uS */
#define COMMAND_DEBOUNCE_VALUE  10*4 /* x100 uS */

#define BUTTON_DEBOUNCE_VALUE  50 /* x1ms */
#define BUTTON_DEBOUNCE_LONG_VALUE  800 /* x1ms */


void sensors_check(void)
{
  uint8_t pin_status;
  uint8_t old_pin_status;

  /* pin1  */
  pin_status = HAL_GPIO_ReadPin(SENSOR_IN_GPIO_Port, SENSOR_IN_Pin) == 0;

  old_pin_status = (buttons_status & (1 << BUTTON_STATUS_MASK_0_SENSOR)) == 0;
  if (pin_status != old_pin_status)
  {
    button_debounce_counter_0 = button_debounce_counter_0 + 1;

    if (button_debounce_counter_0 >= SENSOR_DEBOUNCE_VALUE)
    {
      button_debounce_counter_0 = SENSOR_DEBOUNCE_VALUE;

      // HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, pin_status);
      // HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);

      buttons_status = buttons_status ^ (1 << BUTTON_STATUS_MASK_0_SENSOR);

      if ((buttons_status & (1 << BUTTON_STATUS_MASK_0_SENSOR)) == 0)
      {
        buttons_status_flags = buttons_status_flags | (1 << BUTTON_STATUS_MASK_0_SENSOR);

        if (sensor_counter > 0) {
          sensor_counter = sensor_counter - 1;
        }
      }
    }
  }
  else
  {
    button_debounce_counter_0 = 0;
  }
}

void command_check(void)
{
  uint8_t pin_status;
  uint8_t old_pin_status;

  /* pin3  */
  pin_status = HAL_GPIO_ReadPin(COMM_IN_GPIO_Port, COMM_IN_Pin) == 0;
  old_pin_status = (buttons_status & (1 << BUTTON_STATUS_MASK_2_COMM_IN)) == 0;
  if (pin_status != old_pin_status)
  {
    button_debounce_counter_2 = button_debounce_counter_2 + 1;

    if (button_debounce_counter_2 >= COMMAND_DEBOUNCE_VALUE)
    {
      button_debounce_counter_2 = COMMAND_DEBOUNCE_VALUE;

      buttons_status = buttons_status ^ (1 << BUTTON_STATUS_MASK_2_COMM_IN);
      
      // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      /* press == 0 */
      /* pressup > 0 and set default buttons_status to 1*/
      if ((buttons_status & (1 << BUTTON_STATUS_MASK_2_COMM_IN)) == 0)
      {
        buttons_status_flags = buttons_status_flags | (1 << BUTTON_STATUS_MASK_2_COMM_IN);
      }
    }
  }
  else
  {
    button_debounce_counter_2 = 0;
  }
}

void buttons_check(void)
{
  uint8_t pin_status;
  uint8_t old_pin_status;
  // uint8_t old_pin_status_long;

  // /* pin1  */
  // pin_status = HAL_GPIO_ReadPin(SENSOR_IN_GPIO_Port, SENSOR_IN_Pin) == 0;
  // old_pin_status = (buttons_status & (1 << BUTTON_STATUS_MASK_0_SENSOR)) == 0;
  // if (pin_status != old_pin_status)
  // {
  //   button_debounce_counter_0 = button_debounce_counter_0 + 1;

  //   if (button_debounce_counter_0 >= BUTTON_DEBOUNCE_VALUE)
  //   {
  //     buttons_status = buttons_status ^ (1 << BUTTON_STATUS_MASK_0_SENSOR);

  //     if ((buttons_status & (1 << BUTTON_STATUS_MASK_0_SENSOR)) == 0)
  //     {
  //       buttons_status_flags = buttons_status_flags | (1 << BUTTON_STATUS_MASK_0_SENSOR);
  //     }
  //   }
  // }
  // else
  // {
  //   button_debounce_counter_0 = 0;
  // }

  /* pin2  */
  pin_status = HAL_GPIO_ReadPin(BUTTON_0_GPIO_Port, BUTTON_0_Pin) == 0;


  old_pin_status = (buttons_status & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) == 0;
  if (pin_status != old_pin_status)
  {
    button_debounce_counter_1 = button_debounce_counter_1 + 1;

    if (button_debounce_counter_1 >= BUTTON_DEBOUNCE_VALUE)
    {
      // button_debounce_counter_1 = BUTTON_DEBOUNCE_VALUE;

      buttons_status = buttons_status ^ (1 << BUTTON_STATUS_MASK_1_BUTTON_0);

      /* pressup */
      // if (pin_status == 0) {
      // if ((buttons_status & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) == 1)
      if ((buttons_status & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) > 0)
      {

        /* long press override */
        // if ((buttons_status_long & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) == 0)
        if ((buttons_status_long & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) > 0)
        {
          buttons_status_long = buttons_status_long & ~(1 << BUTTON_STATUS_MASK_1_BUTTON_0); /* clear bit */

          buttons_status_flags_long = buttons_status_flags_long | (1 << BUTTON_STATUS_MASK_1_BUTTON_0);
        } else {
          /* regular scenario */
          buttons_status_flags = buttons_status_flags | (1 << BUTTON_STATUS_MASK_1_BUTTON_0);
        }
      }
      // }
    }
  }

  else
  {
    button_debounce_counter_1 = 0;
  }


  /* LONG pressdown */
  // old_pin_status_long = (buttons_status_long & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) == 0;
  // if (pin_status != old_pin_status_long)
  if (pin_status == 1)
  {
    button_debounce_counter_1_long = button_debounce_counter_1_long + 1;

    if (button_debounce_counter_1_long >= BUTTON_DEBOUNCE_LONG_VALUE)
    {
      button_debounce_counter_1_long = BUTTON_DEBOUNCE_LONG_VALUE;

      // buttons_status_long = buttons_status_long ^ (1 << BUTTON_STATUS_MASK_1_BUTTON_0);
      // buttons_status_long = buttons_status_long & ~(1 << BUTTON_STATUS_MASK_1_BUTTON_0);
      buttons_status_long = buttons_status_long | (1 << BUTTON_STATUS_MASK_1_BUTTON_0);

      /* press == 0 */
      /* pressup > 0 */
      /* see short pressup */
      // if ((buttons_status_long & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) == 0)
      // {
      //   // buttons_status_flags_long = buttons_status_flags_long & ~(1 << BUTTON_STATUS_MASK_1_BUTTON_0); /* clear bit */

      //   buttons_status_flags_long = buttons_status_flags_long | (1 << BUTTON_STATUS_MASK_1_BUTTON_0);
      // }
    }
  }

  else
  {
    button_debounce_counter_1_long = 0;
  }


  // /* pin3  */
  // pin_status = HAL_GPIO_ReadPin(COMM_IN_GPIO_Port, COMM_IN_Pin) == 0;
  // old_pin_status = (buttons_status & (1 << BUTTON_STATUS_MASK_2_COMM_IN)) == 0;
  // if (pin_status != old_pin_status)
  // {
  //   button_debounce_counter_2 = button_debounce_counter_2 + 1;

  //   if (button_debounce_counter_2 >= BUTTON_DEBOUNCE_VALUE)
  //   {
  //     buttons_status = buttons_status ^ (1 << BUTTON_STATUS_MASK_2_COMM_IN);
      
  //     // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  //     if ((buttons_status & (1 << BUTTON_STATUS_MASK_2_COMM_IN)) == 0)
  //     {
  //       buttons_status_flags = buttons_status_flags | (1 << BUTTON_STATUS_MASK_2_COMM_IN);
  //     }
  //   }
  // }
  // else
  // {
  //   button_debounce_counter_2 = 0;
  // }
  //   // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/* ---------------------------------- */
void Reel_motor_stop(void)
{
  HAL_GPIO_WritePin(MOTOR_2_A_GPIO_Port, MOTOR_2_A_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_2_B_GPIO_Port, MOTOR_2_B_Pin, GPIO_PIN_RESET);
}

void Reel_motor_forward(void)
{
  HAL_GPIO_WritePin(MOTOR_2_A_GPIO_Port, MOTOR_2_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_2_B_GPIO_Port, MOTOR_2_B_Pin, GPIO_PIN_RESET);
}

void Reel_motor_reverse(void)
{
  HAL_GPIO_WritePin(MOTOR_2_A_GPIO_Port, MOTOR_2_A_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_2_B_GPIO_Port, MOTOR_2_B_Pin, GPIO_PIN_SET);
}


/* ---------------------------------- */
void Tape_motor_stop(void)
{
  HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_RESET);
}

void Tape_motor_forward(void)
{
  HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_SET);
}

// void Tape_motor_reverse(void)
// {
//   HAL_GPIO_WritePin(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, GPIO_PIN_RESET);
// }

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);

  HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_SET);
  Tape_motor_forward();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
    // HAL_Delay(1000);



    if (state_machine_status == STATE_IDLE) {
      if ((buttons_status_flags & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) != 0)
      {
        buttons_status_flags = buttons_status_flags & ~(1 << BUTTON_STATUS_MASK_1_BUTTON_0); /* clear bit */

        state_machine_status = STATE_MOVE_REEL_FORWARD;
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);
        sensor_counter = SENSOR_COUNTER_MAX;
        Tape_motor_stop();
        Reel_motor_forward();
      }
      if ((buttons_status_flags_long & (1 << BUTTON_STATUS_MASK_1_BUTTON_0)) != 0)
      {
        buttons_status_flags_long = buttons_status_flags_long & ~(1 << BUTTON_STATUS_MASK_1_BUTTON_0); /* clear bit */

        state_machine_status = STATE_MOVE_REEL_BACKWARD;
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);
        sensor_counter = 1; /* 1 is minimum value = 2mm for now */
        Tape_motor_stop();
        Reel_motor_reverse();
      }
      
      if ((buttons_status_flags & (1 << BUTTON_STATUS_MASK_2_COMM_IN)) != 0)
      {
        buttons_status_flags = buttons_status_flags & ~(1 << BUTTON_STATUS_MASK_2_COMM_IN); /* clear bit */

        state_machine_status = STATE_MOVE_REEL_FORWARD;
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);
        sensor_counter = SENSOR_COUNTER_MAX;
        Tape_motor_stop();
        Reel_motor_forward();
      }

    } 
    if (state_machine_status == STATE_MOVE_REEL_FORWARD) {
      if (sensor_counter == 0) {
        Reel_motor_stop();
        HAL_Delay(5);
        Tape_motor_forward();
        state_machine_status = STATE_IDLE;
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_SET);
      }
    } if (state_machine_status == STATE_MOVE_REEL_BACKWARD) {
      if (sensor_counter == 0) {
        Reel_motor_stop();
        HAL_Delay(5);
        Tape_motor_forward();
        state_machine_status = STATE_IDLE;
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_SET);
      }
    }


    // /* process buttons ------- >>>>> */
    // if ((buttons_status_flags & (1<<BUTTON_STATUS_MASK_0_SENSOR)) != 0) {
    //   buttons_status_flags = buttons_status_flags & ~(1<<BUTTON_STATUS_MASK_0_SENSOR); /* clear bit */
    // }

    // if ((buttons_status_flags & (1<<BUTTON_STATUS_MASK_1_BUTTON_0)) != 0) {
    //   buttons_status_flags = buttons_status_flags & ~(1<<BUTTON_STATUS_MASK_1_BUTTON_0); /* clear bit */

    //   // HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
    //   Reel_motor_stop();
    //   Reel_motor_forward();
    //   HAL_Delay(1000);
    //   Reel_motor_stop();
    //   // HAL_Delay(1000);
    //   // Reel_motor_reverse();
    //   // HAL_Delay(1000);
    //   // Reel_motor_stop();



    //   // Tape_motor_stop();
    //   // Tape_motor_forward();
    //   // HAL_Delay(1000);
    //   // Tape_motor_stop();
    //   // HAL_Delay(1000);
    //   // // Tape_motor_reverse();
    //   // // HAL_Delay(1000);
    //   // Tape_motor_stop();
    // }
    // if ((buttons_status_flags_long & (1<<BUTTON_STATUS_MASK_1_BUTTON_0)) != 0) {
    //   buttons_status_flags_long = buttons_status_flags_long & ~(1<<BUTTON_STATUS_MASK_1_BUTTON_0); /* clear bit */

    //   // HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
    //   Reel_motor_stop();
    //   Reel_motor_reverse();
    //   HAL_Delay(1000);
    //   Reel_motor_stop();



    //   // Tape_motor_stop();
    //   // Tape_motor_forward();
    //   // HAL_Delay(1000);
    //   // Tape_motor_stop();
    //   // HAL_Delay(1000);
    //   // // Tape_motor_reverse();
    //   // // HAL_Delay(1000);
    //   // Tape_motor_stop();
    // }
    // if ((buttons_status_flags & (1<<BUTTON_STATUS_MASK_2_COMM_IN)) != 0) {
    //   buttons_status_flags = buttons_status_flags & ~(1<<BUTTON_STATUS_MASK_2_COMM_IN); /* clear bit */
      
    //   // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_2_A_Pin|MOTOR_2_B_Pin|MOTOR_1_A_Pin|COMM_OUT_Pin
                          |MOTOR_1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENSOR_IN_Pin BUTTON_0_Pin COMM_IN_Pin */
  GPIO_InitStruct.Pin = SENSOR_IN_Pin|BUTTON_0_Pin|COMM_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_2_A_Pin MOTOR_2_B_Pin MOTOR_1_A_Pin MOTOR_1_B_Pin */
  GPIO_InitStruct.Pin = MOTOR_2_A_Pin|MOTOR_2_B_Pin|MOTOR_1_A_Pin|MOTOR_1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COMM_OUT_Pin */
  GPIO_InitStruct.Pin = COMM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(COMM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_0_Pin */
  GPIO_InitStruct.Pin = LED_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


uint16_t sensors_tim_prescaler = 0;
uint16_t buttons_tim_prescaler = 0;

/* 10 kHz */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  sensors_tim_prescaler++;
  buttons_tim_prescaler++;
  // if (buttons_tim_prescaler == 10*1000) {
  if (buttons_tim_prescaler == 10*1) {
    buttons_tim_prescaler = 0;
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    
    if (state_machine_status == STATE_IDLE) {
      buttons_check();
    }
  }

  
  if (sensors_tim_prescaler == 1*1) {
    sensors_tim_prescaler = 0;
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    sensors_check();

    if (state_machine_status == STATE_IDLE) {
      command_check();
    }
  }
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
