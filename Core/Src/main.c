/* USER CODE BEGIN Header */ //3
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STEER_KEEP_LAST = 0,
  STEER_HARD_LEFT,
  STEER_LEFT,
  STEER_SOFT_LEFT,
  STEER_STRAIGHT,
  STEER_SOFT_RIGHT,
  STEER_RIGHT,
  STEER_HARD_RIGHT
} steer_cmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_TIM               (&htim2)
#define SERVO_CH                TIM_CHANNEL_1 //

#define MOTOR_TIM               (&htim3)
#define MOTOR_L_PWM_CH          TIM_CHANNEL_1 //
#define MOTOR_R_PWM_CH          TIM_CHANNEL_2 //

#define IR_BLACK_IS_LOW          1U //
#define IR_SENSOR_COUNT          6U //

#define MOTOR_SPEED_STOP          0U //
#define MOTOR_SPEED_STARTUP      40U //
#define MOTOR_SPEED_STRAIGHT     30U //
#define MOTOR_SPEED_SOFT_TURN    20U //
#define MOTOR_SPEED_HARD_TURN    20U //

/* old:
#define SERVO_STRAIGHT_ANGLE     90.0f //
#define SERVO_HARD_LEFT_ANGLE    50.0f //
#define SERVO_LEFT_ANGLE         65.0f //
#define SERVO_SOFT_LEFT_ANGLE    78.0f //
#define SERVO_SOFT_RIGHT_ANGLE  102.0f //
#define SERVO_RIGHT_ANGLE       115.0f //
#define SERVO_HARD_RIGHT_ANGLE  130.0f //
*/
#define SERVO_STRAIGHT_ANGLE    160.0f //
#define SERVO_HARD_LEFT_ANGLE    40.0f //
#define SERVO_LEFT_ANGLE        60.0f //
#define SERVO_SOFT_LEFT_ANGLE   100.0f //
#define SERVO_SOFT_RIGHT_ANGLE  172.0f //
#define SERVO_RIGHT_ANGLE       185.0f //
#define SERVO_HARD_RIGHT_ANGLE  200.0f //

#define OBSTACLE_STOP_DISTANCE_MM   0U //

#define MOTOR_L_FORWARD_LEVEL   GPIO_PIN_SET   //
#define MOTOR_R_FORWARD_LEVEL   GPIO_PIN_RESET //

// Using all 6 sensors: IR1=-5, IR2=-3, IR3=-1, IR4=+1, IR5=+3, IR6=+5
#define STEER_SENSOR_MASK       0x3FU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static GPIO_TypeDef* const ir_ports[IR_SENSOR_COUNT] =
{
  IR_in_1_GPIO_Port,
  IR_in_2_GPIO_Port,
  IR_in_3_GPIO_Port,
  IR_in_4_GPIO_Port,
  IR_in_5_GPIO_Port,
  IR_in_6_GPIO_Port
};

static const uint16_t ir_pins[IR_SENSOR_COUNT] =
{
  IR_in_1_Pin,
  IR_in_2_Pin,
  IR_in_3_Pin,
  IR_in_4_Pin,
  IR_in_5_Pin,
  IR_in_6_Pin
};

volatile uint8_t g_ir_mask = 0U;
volatile uint32_t g_us_mm = 0U;
static float g_last_steer_angle = SERVO_STRAIGHT_ANGLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void servo_set_pulse_us(uint16_t pulse_us);
static void servo_set_angle(float angle);

static void motor_set_raw(uint32_t channel, uint8_t value);
static void motor_set_left(uint8_t speed);
static void motor_set_right(uint8_t speed);
static void motor_set_both(uint8_t speed);
static void motor_stop(void);
static void motor_set_forward_direction(void);

static bool ir_read_black(uint8_t idx);
static uint8_t ir_read_mask(void);

static void us_pin_output(void);
static void us_pin_input(void);
static void us_delay_us(uint16_t us);
static void us_trigger(void);
static uint32_t us_echo_pulse_us(uint32_t timeout_us);
static uint32_t us_read_distance_mm(void);

static steer_cmd_t steering_get_command(uint8_t ir_mask);
static void steering_apply_command(steer_cmd_t cmd);
static uint8_t speed_for_steering(steer_cmd_t cmd);
static void control_loop(void);

static void servo_sweep_test(void);
static void motor_test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================================ Servo =================================== */
/* OLD:
static void servo_set_pulse_us(uint16_t pulse_us)
{
  if (pulse_us < 1000U) pulse_us = 1000U;
  if (pulse_us > 2000U) pulse_us = 2000U;

  __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_CH, pulse_us);
}
*/
static void servo_set_pulse_us(uint16_t pulse_us)
{
  if (pulse_us < 800U)  pulse_us = 800U;
  if (pulse_us > 2400U) pulse_us = 2400U;

  __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_CH, pulse_us);
}

/* OLD:
static void servo_set_angle(float angle)
{
  if (angle < 0.0f) angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;

  {
    float pulse_us_f = 1000.0f + (angle / 180.0f) * 1000.0f;
    servo_set_pulse_us((uint16_t)(pulse_us_f + 0.5f));
  }
}
*/
static void servo_set_angle(float angle)
{
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 250.0f) angle = 250.0f;

  {
    float pulse_us_f = 1000.0f + (angle / 180.0f) * 1000.0f;
    servo_set_pulse_us((uint16_t)(pulse_us_f + 0.5f));
  }
}

/* ================================ Motor =================================== */
static void motor_set_raw(uint32_t channel, uint8_t value)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(MOTOR_TIM);
  uint32_t pulse = ((uint32_t)value * arr) / 255U;
  __HAL_TIM_SET_COMPARE(MOTOR_TIM, channel, pulse);
}

static void motor_set_left(uint8_t speed)
{
  motor_set_raw(MOTOR_L_PWM_CH, speed);
}

static void motor_set_right(uint8_t speed)
{
  motor_set_raw(MOTOR_R_PWM_CH, speed);
}

static void motor_set_both(uint8_t speed)
{
  motor_set_left(speed);
  motor_set_right(speed);
}

static void motor_stop(void)
{
  motor_set_both(MOTOR_SPEED_STOP);
}

static void motor_set_forward_direction(void)
{
  HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, MOTOR_L_FORWARD_LEVEL);
  HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, MOTOR_R_FORWARD_LEVEL);
}

/* ================================ IR Sensors ============================== */
static bool ir_read_black(uint8_t idx)
{
  if (idx >= IR_SENSOR_COUNT)
  {
    return false;
  }

  GPIO_PinState s = HAL_GPIO_ReadPin(ir_ports[idx], ir_pins[idx]);

#if IR_BLACK_IS_LOW
  return (s == GPIO_PIN_RESET);
#else
  return (s == GPIO_PIN_SET);
#endif
}

static uint8_t ir_read_mask(void)
{
  uint8_t mask = 0U;

  for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++)
  {
    if (ir_read_black(i))
    {
      mask |= (1U << i);
    }
  }

  return mask;
}

/* ============================= Ultrasonic ================================= */
static void us_pin_output(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = US_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_GPIO_Port, &GPIO_InitStruct);
}

static void us_pin_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = US_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(US_GPIO_Port, &GPIO_InitStruct);
}

static void us_delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  while (__HAL_TIM_GET_COUNTER(&htim4) < us)
  {
  }
}

static void us_trigger(void)
{
  us_pin_output();

  HAL_GPIO_WritePin(US_GPIO_Port, US_Pin, GPIO_PIN_RESET);
  us_delay_us(2);

  HAL_GPIO_WritePin(US_GPIO_Port, US_Pin, GPIO_PIN_SET);
  us_delay_us(10);

  HAL_GPIO_WritePin(US_GPIO_Port, US_Pin, GPIO_PIN_RESET);

  us_pin_input();
}

static uint32_t us_echo_pulse_us(uint32_t timeout_us)
{
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  while (HAL_GPIO_ReadPin(US_GPIO_Port, US_Pin) == GPIO_PIN_RESET)
  {
    if (__HAL_TIM_GET_COUNTER(&htim4) > timeout_us)
    {
      return 0U;
    }
  }

  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  while (HAL_GPIO_ReadPin(US_GPIO_Port, US_Pin) == GPIO_PIN_SET)
  {
    if (__HAL_TIM_GET_COUNTER(&htim4) > timeout_us)
    {
      return 0U;
    }
  }

  return __HAL_TIM_GET_COUNTER(&htim4);
}

static uint32_t us_read_distance_mm(void)
{
  uint32_t pw;

  us_trigger();
  pw = us_echo_pulse_us(30000U);

  if (pw == 0U)
  {
    return 0U;
  }

  return (pw * 343U) / 2000U;
}

/* ============================== Steering ================================= */
static steer_cmd_t steering_get_command(uint8_t ir_mask)
{
  uint8_t steer_mask = ir_mask & STEER_SENSOR_MASK;

  if (steer_mask == 0U)
  {
    return STEER_KEEP_LAST;
  }

  /* IR1=-5, IR2=-3, IR3=-1, IR4=+1, IR5=+3, IR6=+5 */
  int32_t weighted_sum = 0;
  uint32_t count = 0U;

  if (steer_mask & (1U << 0)) { weighted_sum += -5; count++; }
  if (steer_mask & (1U << 1)) { weighted_sum += -3; count++; }
  if (steer_mask & (1U << 2)) { weighted_sum += -1; count++; }
  if (steer_mask & (1U << 3)) { weighted_sum +=  1; count++; }
  if (steer_mask & (1U << 4)) { weighted_sum +=  3; count++; }
  if (steer_mask & (1U << 5)) { weighted_sum +=  5; count++; }

  if (count == 0U)
  {
    return STEER_KEEP_LAST;
  }

  {
    int32_t error = weighted_sum / (int32_t)count;

    if (error <= -4) return STEER_HARD_LEFT;
    if (error <= -2) return STEER_LEFT;
    if (error == -1) return STEER_SOFT_LEFT;
    if (error == 0)  return STEER_STRAIGHT;
    if (error == 1)  return STEER_SOFT_RIGHT;
    if (error >= 2 && error < 4) return STEER_RIGHT;
    if (error >= 4) return STEER_HARD_RIGHT;

    return STEER_STRAIGHT;
  }
}

static void steering_apply_command(steer_cmd_t cmd)
{
  float target_angle = g_last_steer_angle;

  switch (cmd)
  {
    case STEER_HARD_LEFT:  target_angle = SERVO_HARD_LEFT_ANGLE;  break;
    case STEER_LEFT:       target_angle = SERVO_LEFT_ANGLE;       break;
    case STEER_SOFT_LEFT:  target_angle = SERVO_SOFT_LEFT_ANGLE;  break;
    case STEER_STRAIGHT:   target_angle = SERVO_STRAIGHT_ANGLE;   break;
    case STEER_SOFT_RIGHT: target_angle = SERVO_SOFT_RIGHT_ANGLE; break;
    case STEER_RIGHT:      target_angle = SERVO_RIGHT_ANGLE;      break;
    case STEER_HARD_RIGHT: target_angle = SERVO_HARD_RIGHT_ANGLE; break;
    case STEER_KEEP_LAST:
    default:
      break;
  }

  g_last_steer_angle = target_angle;
  servo_set_angle(target_angle);
}

static uint8_t speed_for_steering(steer_cmd_t cmd)
{
  switch (cmd)
  {
    case STEER_HARD_LEFT:
    case STEER_HARD_RIGHT:
      return MOTOR_SPEED_HARD_TURN;

    case STEER_LEFT:
    case STEER_RIGHT:
    case STEER_SOFT_LEFT:
    case STEER_SOFT_RIGHT:
      return MOTOR_SPEED_SOFT_TURN;

    case STEER_STRAIGHT:
      return MOTOR_SPEED_STRAIGHT;

    case STEER_KEEP_LAST:
    default:
      return MOTOR_SPEED_STRAIGHT;
  }
}

static void control_loop(void)
{
  steer_cmd_t steer_cmd;
  uint8_t motor_speed;

  g_ir_mask = ir_read_mask();
  g_us_mm = us_read_distance_mm();

  steer_cmd = steering_get_command(g_ir_mask);
  motor_speed = speed_for_steering(steer_cmd);

  if ((OBSTACLE_STOP_DISTANCE_MM > 0U) &&
      (g_us_mm > 0U) &&
      (g_us_mm < OBSTACLE_STOP_DISTANCE_MM))
  {
    motor_stop();
  }
  else
  {
    motor_set_forward_direction();
    motor_set_both(motor_speed);
  }

  steering_apply_command(steer_cmd);

  HAL_Delay(10);
}

/* ================================ Tests =================================== */
static void servo_sweep_test(void)
{
  servo_set_angle(120.0f); // HARD LEFT
  HAL_Delay(1000);

  servo_set_angle(135.0f); // LEFT
  HAL_Delay(1000);

  servo_set_angle(148.0f); // SOFT LEFT
  HAL_Delay(1000);

  servo_set_angle(160.0f); // CENTRE
  HAL_Delay(1000);

  servo_set_angle(172.0f); // SOFT RIGHT
  HAL_Delay(1000);

  servo_set_angle(185.0f); // RIGHT
  HAL_Delay(1000);

  servo_set_angle(200.0f); // HARD RIGHT
  HAL_Delay(1000);
}

static void motor_test(void)
{
  HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 200);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* USER CODE BEGIN Init */
  HAL_Init();
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim4);

  servo_set_angle(SERVO_STRAIGHT_ANGLE);
  motor_set_forward_direction();
  motor_stop();

  HAL_Delay(200);

  motor_set_both(MOTOR_SPEED_STARTUP);
  HAL_Delay(100);
  motor_stop();
  HAL_Delay(100);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */

	control_loop();

//    servo_sweep_test();
//    motor_test();

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialisation Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialisation Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialisation Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialisation Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialisation Function
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
  * @brief GPIO Initialisation Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, MOTOR_L_DIR_Pin|MOTOR_R_DIR_Pin|US_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IR_in_1_Pin|IR_in_2_Pin|IR_in_3_Pin|IR_in_4_Pin
                      | IR_in_5_Pin|IR_in_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MOTOR_L_DIR_Pin|MOTOR_R_DIR_Pin|US_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
