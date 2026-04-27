/* USER CODE BEGIN Header */
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
// Servo timer
#define SERVO_TIM               (&htim2)
#define SERVO_CH                TIM_CHANNEL_1 // PA0

// Motor timer
#define MOTOR_TIM               (&htim3)
#define MOTOR_L_PWM_CH          TIM_CHANNEL_1 // PA6
#define MOTOR_R_PWM_CH          TIM_CHANNEL_2 // PA7

// Sensor count
#define IR_BLACK_IS_LOW         0U // 0 if read black, 1 if white
#define IR_SENSOR_COUNT         6U // 6 IR sensors total, PC0 - PC5

// Initial Motor Speeds
#define MOTOR_SPEED_STOP         0U // Stationary speed,
#define MOTOR_SPEED_STARTUP     70U // Kick-start speed,

// US sensor stopping distance
#define OBSTACLE_STOP_DISTANCE_MM   100U // 100 mm, PB7

// Motor directions
#define MOTOR_L_FORWARD_LEVEL   GPIO_PIN_SET   // SET forward, RESET back, PB4
#define MOTOR_R_FORWARD_LEVEL   GPIO_PIN_RESET // RESET forward, SET back, PB5

// Define track: 1U = Track 1 | 2U = Track 2
#define TRACK_PROFILE  2U

/*================================== Track 1 =================================*/
#if (TRACK_PROFILE == 1U)

// Motor Speeds
#define MOTOR_SPEED_STRAIGHT       50U
#define MOTOR_SPEED_SOFT_TURN      45U
#define MOTOR_SPEED_HARD_TURN      45U

// Servo turn ang les
#define SERVO_STRAIGHT_ANGLE     90.0f
#define SERVO_HARD_LEFT_ANGLE    20.0f
#define SERVO_LEFT_ANGLE         40.0f
#define SERVO_SOFT_LEFT_ANGLE    65.0f
#define SERVO_SOFT_RIGHT_ANGLE  115.0f
#define SERVO_RIGHT_ANGLE       140.0f
#define SERVO_HARD_RIGHT_ANGLE  160.0f

// Sensor hyperparameters
#define IR_SAMPLE_COUNT             5U // Samples in a cycle, Higher  more noise filtering
#define IR_SAMPLE_DELAY_US        200U // Time (µs) between consec samples, Larger = more stable
#define STEER_CMD_CONFIRM_COUNT     1U // Number of consecutive identical decisions required
#define SERVO_MAX_STEP_PER_LOOP  10.0f // Maximum angle change per control loop
#define CONTROL_LOOP_DELAY_MS      10U // Delay between control updates (ms)

// Active sensor count per direction
#define STEER_LEFT_SENSOR_COUNT  1U // 1 IR sensor left
#define STEER_RIGHT_SENSOR_COUNT 1U // 1 IR sensor right

/*================================== Track 2 =================================*/
#elif (TRACK_PROFILE == 2U)

// Motor Speeds
#define MOTOR_SPEED_STRAIGHT       40U
#define MOTOR_SPEED_SOFT_TURN      45U
#define MOTOR_SPEED_HARD_TURN      45U

// Servo turn angles
#define SERVO_STRAIGHT_ANGLE     90.0f
#define SERVO_HARD_LEFT_ANGLE     5.0f
#define SERVO_LEFT_ANGLE         25.0f
#define SERVO_SOFT_LEFT_ANGLE    45.0f
#define SERVO_SOFT_RIGHT_ANGLE  135.0f
#define SERVO_RIGHT_ANGLE       155.0f
#define SERVO_HARD_RIGHT_ANGLE  175.0f

// Sensor hyperparameters
#define IR_SAMPLE_COUNT             3U // Samples in a cycle, Higher  more noise filtering
#define IR_SAMPLE_DELAY_US         50U // Time (µs) between consec samples, Larger = more stable
#define STEER_CMD_CONFIRM_COUNT     1U // Number of consecutive identical decisions required
#define SERVO_MAX_STEP_PER_LOOP  15.0f // Maximum angle change per control loop
#define CONTROL_LOOP_DELAY_MS       1U // Delay between control updates (ms)

// Active sensor count per direction
#define STEER_LEFT_SENSOR_COUNT  2U // 2 IR sensor left
#define STEER_RIGHT_SENSOR_COUNT 2U // 2 IR sensor right

#else
#error "TRACK_PROFILE must be 1 or 2!"
#endif
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
// GPIO mapping for IR1..IR6
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

#if (TRACK_PROFILE == 1U)
// Track 1: IR2 on left, IR5 on right
static const uint8_t steer_left_indices[STEER_LEFT_SENSOR_COUNT]   = {1U}; // IR2
static const uint8_t steer_right_indices[STEER_RIGHT_SENSOR_COUNT] = {4U}; // IR5
#elif (TRACK_PROFILE == 2U)
// Track 2: IR2+IR3 on left, IR4+IR5 on right
static const uint8_t steer_left_indices[STEER_LEFT_SENSOR_COUNT]   = {1U, 2U}; // IR2, IR3
static const uint8_t steer_right_indices[STEER_RIGHT_SENSOR_COUNT] = {3U, 4U}; // IR4, IR5
#endif

// Latest sensor and steering state
volatile uint8_t g_ir_mask = 0U;
volatile uint32_t g_us_mm = 0U;
static float g_last_steer_angle = SERVO_STRAIGHT_ANGLE;

static steer_cmd_t g_last_cmd = STEER_STRAIGHT;
static steer_cmd_t g_pending_cmd = STEER_STRAIGHT;
static uint8_t g_pending_count = 0U;
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
// Servo control helpers
static void servo_set_pulse_us(uint16_t pulse_us);
static void servo_set_angle(float angle);

// Motor control helpers
static void motor_set_raw(uint32_t channel, uint8_t value);
static void motor_set_left(uint8_t speed);
static void motor_set_right(uint8_t speed);
static void motor_set_both(uint8_t speed);
static void motor_stop(void);
static void motor_set_forward_direction(void);

// IR sensor reading helpers
static bool ir_read_black(uint8_t idx);
static uint8_t ir_read_mask(void);

// US sensor helpers
static void us_pin_output(void);
static void us_pin_input(void);
static void us_delay_us(uint16_t us);
static void us_trigger(void);
static uint32_t us_echo_pulse_us(uint32_t timeout_us);
static uint32_t us_read_distance_mm(void);

// Steering decision helpers
static steer_cmd_t steering_get_command(uint8_t ir_mask);
static steer_cmd_t steering_filter_command(steer_cmd_t new_cmd);
static void steering_apply_command(steer_cmd_t cmd);
static uint8_t speed_for_steering(steer_cmd_t cmd);
static void control_loop(void);

// Manual test helpers
static void servo_sweep_test(void);
static void motor_test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*================================= Servo ====================================*/
// Servo PWM pulse width in microseconds
static void servo_set_pulse_us(uint16_t pulse_us)
{
  // Clamp pulse width to safe servo range
  if (pulse_us < 800U)  pulse_us = 800U;
  if (pulse_us > 2400U) pulse_us = 2400U;

  __HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_CH, pulse_us);
}

// Convert steering angle to PWM pulse and send it to the servo
static void servo_set_angle(float angle)
{
  // Clamp angle to standard servo range (0–180 degrees, 90 is centre)
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;

  // Convert angle to PWM pulse
  float pulse_us_f = 1000.0f + (angle / 180.0f) * 1000.0f;

  servo_set_pulse_us((uint16_t)(pulse_us_f + 0.5f));
}

/*================================= Motor ====================================*/
// Write raw PWM value to one motor channel
static void motor_set_raw(uint32_t channel, uint8_t value)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(MOTOR_TIM);
  uint32_t pulse = ((uint32_t)value * arr) / 255U;
  __HAL_TIM_SET_COMPARE(MOTOR_TIM, channel, pulse);
}
// Set left motor speed
static void motor_set_left(uint8_t speed)
{
  motor_set_raw(MOTOR_L_PWM_CH, speed);
}

// Set right motor speed
static void motor_set_right(uint8_t speed)
{
  motor_set_raw(MOTOR_R_PWM_CH, speed);
}

// Set both motors to the same speed
static void motor_set_both(uint8_t speed)
{
  motor_set_left(speed);
  motor_set_right(speed);
}

// Stop both motors
static void motor_stop(void)
{
  motor_set_both(MOTOR_SPEED_STOP);
}

// Set motor direction pins for forward travel
static void motor_set_forward_direction(void)
{
  HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, MOTOR_L_FORWARD_LEVEL);
  HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, MOTOR_R_FORWARD_LEVEL);
}

/*===================================== IR ===================================*/
// Read one IR sensor and return true if it detects black
static bool ir_read_black(uint8_t idx)
{
  // Check for valid sensor index
  if (idx >= IR_SENSOR_COUNT)
  {
    return false;   // Invalid index → no detection
  }
  // Read the GPIO pin state for the given IR sensor
  GPIO_PinState s = HAL_GPIO_ReadPin(ir_ports[idx], ir_pins[idx]);

#if IR_BLACK_IS_LOW
  return (s == GPIO_PIN_RESET);
#else
  return (s == GPIO_PIN_SET);
#endif
}

// Read all IR sensors and pack results into a bit mask
static uint8_t ir_read_mask(void)
{
  uint8_t mask = 0U; // Bitmask to store results for all sensors

  // Loop through each IR sensor
  for (uint8_t i = 0U; i < IR_SENSOR_COUNT; i++)
  {
    uint8_t black_count = 0U; // Count how many times black is detected

    // Take multiple samples for noise reduction
    for (uint8_t s = 0U; s < IR_SAMPLE_COUNT; s++)
    {
      // Read sensor and increment count if black is detected
      if (ir_read_black(i))
      {
        black_count++;
      }

      // Small delay between samples
      us_delay_us(IR_SAMPLE_DELAY_US);
    }

    // Majority voting: if more than half the samples detect black
    if (black_count >= ((IR_SAMPLE_COUNT / 2U) + 1U))
    {
      // Set the corresponding bit in the mask
      mask |= (1U << i);
    }
  }

  return mask;
}

/*============================== Ultrasonic ==================================*/
// Configure shared US pin as output for trigger pulse
static void us_pin_output(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = US_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 	 // Push-pull output mode
  GPIO_InitStruct.Pull = GPIO_NOPULL; 			 // No pull-up or pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   // Low speed is sufficient
  HAL_GPIO_Init(US_GPIO_Port, &GPIO_InitStruct);
}

// Configure shared US pin as input for echo reading
static void us_pin_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = US_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;	// Set pin as input to read echo signal
  GPIO_InitStruct.Pull = GPIO_NOPULL;		// No pull resistor needed
  HAL_GPIO_Init(US_GPIO_Port, &GPIO_InitStruct);
}

// Microsecond delay using TIM4
static void us_delay_us(uint16_t us)
{
  // Reset timer counter
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  // Wait until counter reaches desired microseconds
  while (__HAL_TIM_GET_COUNTER(&htim4) < us)
  {
  }
}

// Send trigger pulse to US sensor
static void us_trigger(void)
{
  // Set pin as output to send trigger signal
  us_pin_output();

  // Ensure clean LOW before pulse
  HAL_GPIO_WritePin(US_GPIO_Port, US_Pin, GPIO_PIN_RESET);
  us_delay_us(2U);

  // Send HIGH pulse (minimum 10 µs required by most sensors)
  HAL_GPIO_WritePin(US_GPIO_Port, US_Pin, GPIO_PIN_SET);
  us_delay_us(10U);

  // End trigger pulse
  HAL_GPIO_WritePin(US_GPIO_Port, US_Pin, GPIO_PIN_RESET);

  // Switch pin back to input mode to listen for echo
  us_pin_input();
}

// Measure echo pulse width in microseconds
static uint32_t us_echo_pulse_us(uint32_t timeout_us)
{
  // Wait for echo signal to go HIGH (start of echo)
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  while (HAL_GPIO_ReadPin(US_GPIO_Port, US_Pin) == GPIO_PIN_RESET)
  {
	// Timeout protection to avoid infinite loop
    if (__HAL_TIM_GET_COUNTER(&htim4) > timeout_us)
    {
      return 0U; // No echo detected
    }
  }

  // Measure how long the signal stays HIGH (echo duration)
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  while (HAL_GPIO_ReadPin(US_GPIO_Port, US_Pin) == GPIO_PIN_SET)
  {
    // Timeout protection
    if (__HAL_TIM_GET_COUNTER(&htim4) > timeout_us)
    {
      return 0U;
    }
  }

  // Return pulse width in microseconds
  return __HAL_TIM_GET_COUNTER(&htim4);
}

// Read obstacle distance in millimetres (mm)
static uint32_t us_read_distance_mm(void)
{
  uint32_t pw;

  // Send trigger pulse
  us_trigger();

  // Measure echo pulse width (timeout ~30 ms → ~5 meters max range)
  pw = us_echo_pulse_us(30000U);

  // If no valid echo, return 0 (no object or out of range)
  if (pw == 0U)
  {
    return 0U;
  }

  // distance = (time × speed of sound) / 2
  // speed of sound ≈ 343 m/s → 0.343 mm/µs
  // division by 2 accounts for round trip (to object and back)
  return (pw * 343U) / 2000U;
}

/*=============================== Steering ==================================*/
// Generate steering command from active IR sensor states
static steer_cmd_t steering_get_command(uint8_t ir_mask)
{
  uint8_t left_count = 0U;
  uint8_t right_count = 0U;

  // Count how many LEFT-side sensors detect the line
  for (uint8_t i = 0U; i < STEER_LEFT_SENSOR_COUNT; i++)
  {

	// Check valid index and if that sensor bit is active in mask
    uint8_t idx = steer_left_indices[i];
    if ((idx < IR_SENSOR_COUNT) && ((ir_mask & (1U << idx)) != 0U))
    {
      left_count++;
    }
  }

  // Count how many RIGHT-side sensors detect the line
  for (uint8_t i = 0U; i < STEER_RIGHT_SENSOR_COUNT; i++)
  {
    uint8_t idx = steer_right_indices[i];
    if ((idx < IR_SENSOR_COUNT) && ((ir_mask & (1U << idx)) != 0U))
    {
      right_count++;
    }
  }

  // If no sensors detect the line → keep last steering decision
  if ((left_count == 0U) && (right_count == 0U))
  {
    return STEER_KEEP_LAST;
  }

  // Equal detection → go straight
  if (left_count == right_count)
  {
    return STEER_STRAIGHT;
  }

  // More detection on LEFT → steer left
  if (left_count > right_count)
  {
	// Multiple sensors active → stronger turn
    if ((STEER_LEFT_SENSOR_COUNT > 1U) && (left_count >= 2U))
    {
      return STEER_HARD_LEFT;
    }
    else
    {
      return STEER_LEFT;
    }
  }

  // More detection on RIGHT → steer right
  if (right_count > left_count)
  {
	// Multiple sensors active → stronger turn
    if ((STEER_RIGHT_SENSOR_COUNT > 1U) && (right_count >= 2U))
    {
      return STEER_HARD_RIGHT;
    }
    else
    {
      return STEER_RIGHT;
    }
  }

  // Fallback
  return STEER_KEEP_LAST;
}

// Smooth sudden steering command changes
static steer_cmd_t steering_filter_command(steer_cmd_t new_cmd)
{
  // If command is same as last applied → reset pending and return
  if (new_cmd == g_last_cmd)
  {
    g_pending_cmd = new_cmd;
    g_pending_count = 0U;
    return g_last_cmd;
  }

  // If new command matches the pending one → increment confirmation counter
  if (new_cmd == g_pending_cmd)
  {
    if (g_pending_count < STEER_CMD_CONFIRM_COUNT)
    {
      g_pending_count++;
    }
  }
  else
  {
	// New command is different → reset confirmation process
    g_pending_cmd = new_cmd;
    g_pending_count = 0U;
  }

  // Only accept new command after it is stable for N cycles
  if (g_pending_count >= STEER_CMD_CONFIRM_COUNT)
  {
    g_last_cmd = g_pending_cmd;
    g_pending_count = 0U;
  }

  // Return the currently accepted (stable) command
  return g_last_cmd;
}

// Convert steering command into servo movement
static void steering_apply_command(steer_cmd_t cmd)
{
  // Start from last angle (for smooth transitions)
  float target_angle = g_last_steer_angle;

  // Map steering command to predefined servo angles
  switch (cmd)
  {
    case STEER_HARD_LEFT:  target_angle = SERVO_HARD_LEFT_ANGLE;  break;
    case STEER_LEFT:       target_angle = SERVO_LEFT_ANGLE;       break;
    case STEER_SOFT_LEFT:  target_angle = SERVO_SOFT_LEFT_ANGLE;  break;
    case STEER_STRAIGHT:   target_angle = SERVO_STRAIGHT_ANGLE;   break;
    case STEER_SOFT_RIGHT: target_angle = SERVO_SOFT_RIGHT_ANGLE; break;
    case STEER_RIGHT:      target_angle = SERVO_RIGHT_ANGLE;      break;
    case STEER_HARD_RIGHT: target_angle = SERVO_HARD_RIGHT_ANGLE; break;

    // Keep last angle if no change requested
    case STEER_KEEP_LAST:
    default:
      target_angle = g_last_steer_angle;
      break;
  }

  // Limit how fast the servo can move (prevents jerky motion)
  if (target_angle > (g_last_steer_angle + SERVO_MAX_STEP_PER_LOOP))
  {
    target_angle = g_last_steer_angle + SERVO_MAX_STEP_PER_LOOP;
  }
  else if (target_angle < (g_last_steer_angle - SERVO_MAX_STEP_PER_LOOP))
  {
    target_angle = g_last_steer_angle - SERVO_MAX_STEP_PER_LOOP;
  }

  // Update stored angle and apply to servo
  g_last_steer_angle = target_angle;
  servo_set_angle(target_angle);
}

// Choose motor speed based on steering demand
static uint8_t speed_for_steering(steer_cmd_t cmd)
{
  switch (cmd)
  {
    // Sharp turns → slow down significantly
    case STEER_HARD_LEFT:
    case STEER_HARD_RIGHT:
      return MOTOR_SPEED_HARD_TURN;

    // Moderate turns → medium speed
    case STEER_LEFT:
    case STEER_RIGHT:
    case STEER_SOFT_LEFT:
    case STEER_SOFT_RIGHT:
      return MOTOR_SPEED_SOFT_TURN;

    // If unsure, stay cautious
    case STEER_KEEP_LAST:
      return MOTOR_SPEED_SOFT_TURN;

    // Straight → maximum speed
    case STEER_STRAIGHT:
    default:
      return MOTOR_SPEED_STRAIGHT;
  }
}

// Run one full read-decide-act control cycle
static void control_loop(void)
{
  steer_cmd_t steer_cmd;
  uint8_t motor_speed;

  // SENSE
  g_ir_mask = ir_read_mask();      // Read line sensors
  g_us_mm = us_read_distance_mm(); // Read ultrasonic distance

  // DECIDE
  steer_cmd = steering_get_command(g_ir_mask);    // Raw decision
  steer_cmd = steering_filter_command(steer_cmd); // Smoothed decision

  motor_speed = speed_for_steering(steer_cmd);    // Adjust speed

  // OBSTACLE DETECTION
  if ((OBSTACLE_STOP_DISTANCE_MM > 0U) &&
      (g_us_mm > 0U) &&
      (g_us_mm < OBSTACLE_STOP_DISTANCE_MM))
  {
	// Stop if obstacle is too close
    motor_stop();
  }
  else
  {
	// Otherwise move forward at selected speed
    motor_set_forward_direction();
    motor_set_both(motor_speed);
  }

  // Apply steering to servo
  steering_apply_command(steer_cmd);

  // Small delay to control loop frequency
  HAL_Delay(CONTROL_LOOP_DELAY_MS);
}

/*================================= Tests ====================================*/
// Servo sweep test through preset steering positions
static void servo_sweep_test(void)
{
  servo_set_angle(SERVO_HARD_LEFT_ANGLE);
  HAL_Delay(1000);

  servo_set_angle(SERVO_LEFT_ANGLE);
  HAL_Delay(1000);

  servo_set_angle(SERVO_SOFT_LEFT_ANGLE);
  HAL_Delay(1000);

  servo_set_angle(SERVO_STRAIGHT_ANGLE);
  HAL_Delay(1000);

  servo_set_angle(SERVO_SOFT_RIGHT_ANGLE);
  HAL_Delay(1000);

  servo_set_angle(SERVO_RIGHT_ANGLE);
  HAL_Delay(1000);

  servo_set_angle(SERVO_HARD_RIGHT_ANGLE);
  HAL_Delay(1000);
}

// Motor test for L and R separately
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

  servo_set_angle(SERVO_STRAIGHT_ANGLE); // Center steering
  motor_set_forward_direction();
  motor_stop();
  HAL_Delay(200);

  motor_set_both(MOTOR_SPEED_STARTUP); // Kickstart motors
  HAL_Delay(100);
  motor_stop();
  HAL_Delay(100);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    control_loop();

    // Test/debug functions
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
                              | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

}

/**
  * @brief TIM2 Initialisation Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};


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

  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialisation Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};


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

  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialisation Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};


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

}

/**
  * @brief USART2 Initialisation Function
  * @param None
  * @retval None
  */
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialisation Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};


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

}


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

}
#endif /* USE_FULL_ASSERT */
