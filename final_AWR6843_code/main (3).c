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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_PWM_MIN_US             1000
#define RC_PWM_CENTER_US          1500
#define RC_PWM_MAX_US             2000

/* [2. 출력 대역 스케일링] 하드웨어 안전 출력 범위 정의 */
#define THROTTLE_MIN_US           1300
#define THROTTLE_CENTER_US        1500
#define THROTTLE_MAX_US           1700

/* [4. 쓰로틀 지수 커브 필터] 3차 비선형 가중치 (35%) */
#define THROTTLE_EXPO_PERCENT     35

/* [3. 데드밴드 필터] 보고서 명세(8µs)와 일치하도록 수정 (기존 20 -> 8) */
#define THROTTLE_DEADBAND_US      8

/* 추가 입력 노이즈 프리필터 대역 */
#define THROTTLE_INPUT_FILTER_US  10
#define THROTTLE_SLEW_STEP_US     20

/* [5. 역기전력 방지 방향 가드] 중립 계류 시간 (300ms) */
#define THROTTLE_REVERSE_HOLD_MS  300

/* [6. 조향 지터 및 서보 보호 필터] 변동폭 제한 (4µs) */
#define STEERING_JITTER_DEADBAND_US 4

#define MOTOR_ARM_US              THROTTLE_CENTER_US
#define MOTOR_ARM_TIME_MS         3000

#define SWITCH_LEFT_US            1000
#define SWITCH_CENTER_US          1500
#define SWITCH_RIGHT_US           2000
#define SWITCH_DEADBAND_US        100

#define SPI_FRAME_LEN             8
#define SPI_SYNC_0                0xAA
#define SPI_SYNC_1                0x55
#define SPI_CRC8_POLYNOMIAL       0x07
#define VIRTUAL_SPEED_MAX_KMH     50
#define STEERING_MAX_DEG          45
#define TTC_UNAVAILABLE_X100      0xFFFFU

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint32_t IC_Val1 = 0;  // TIM2 - 구동모터 입력
volatile uint32_t IC_Val2 = 0;  // TIM3 - 서보모터 입력
volatile uint32_t IC_Val3 = 0;  // TIM4 - 스위치 입력

static volatile uint32_t throttle_output_us = THROTTLE_CENTER_US;
static volatile uint32_t throttle_neutral_hold_until = 0;
static volatile uint32_t steering_output_us = RC_PWM_CENTER_US;
static uint32_t motor_start_tick = 0;

// SPI1 송수신 버퍼
static uint8_t spi_tx_buffer[SPI_FRAME_LEN] = {0};
static uint8_t spi_rx_buffer[SPI_FRAME_LEN] = {0};
static volatile uint8_t spi_last_tx_buffer[SPI_FRAME_LEN] = {0};
static volatile uint8_t spi_last_rx_buffer[SPI_FRAME_LEN] = {0};
static volatile HAL_StatusTypeDef spi_last_status = HAL_OK;
static volatile uint32_t spi_ok_count = 0;
static volatile uint32_t spi_error_count = 0;
static volatile uint32_t spi_invalid_frame_count = 0;
static volatile uint8_t spi_transfer_armed = 0;
static uint8_t spi_tx_sequence = 0;
static volatile uint8_t spi_last_rx_sequence = 0;
static uint8_t spi_rx_frame[SPI_FRAME_LEN] = {0};
static uint8_t spi_rx_frame_index = 0;

// 깜빡이 상태 추적
static volatile uint8_t blink_state = 0;  // 0=NONE, 1=RIGHT, 2=LEFT
static volatile uint8_t blink_state_prev = 0;  // 상태 변경 감지용
static volatile uint8_t lane_change_allowed = 0;  // 라즈베리파이 응답
static volatile int8_t virtual_speed_kmh = 0;
static volatile int8_t steering_angle_deg = 0;
static volatile uint16_t received_ttc_x100 = TTC_UNAVAILABLE_X100;
static volatile uint8_t rpi_flags = 0;

// SPI 타이밍
static uint32_t spi_last_tx_tick = 0;

// printf 출력을 LPUART1(USB 포트)로 연결
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  // huart1 대신 hlpuart1을 사용합니다.
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static uint32_t Clamp_U32(uint32_t value, uint32_t min, uint32_t max);
static uint32_t Reverse_Rc_Pwm(uint32_t input_us);
static uint32_t Apply_Throttle_Input_Filter(uint32_t input_us);
static uint32_t Apply_Throttle_Curve(uint32_t input_us);
static uint32_t Apply_Throttle_Direction_Guard(uint32_t target_us);
static uint32_t Apply_Throttle_Slew(uint32_t target_us);
static uint32_t Apply_Steering_Jitter_Filter(uint32_t target_us);

/* SPI 통신 함수 */
static uint8_t Get_Blink_State(uint32_t switch_input_us);
static int8_t Calculate_Virtual_Speed_Kmh(uint32_t throttle_us);
static int8_t Calculate_Steering_Angle_Deg(uint32_t steering_us);
static uint16_t Get_U16_LE(const uint8_t *buf);
static uint8_t SPI_Calculate_CRC8(const uint8_t *data, uint8_t length);
static uint8_t SPI_Is_Valid_Frame(const uint8_t *frame);
static void SPI1_Consume_Rx_Byte(uint8_t value);
static void SPI1_Prepare_Tx_Data(void);
static HAL_StatusTypeDef SPI1_Arm_Transfer(void);
static void SPI1_Process_Completed_Transfer(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t Clamp_U32(uint32_t value, uint32_t min, uint32_t max)
{
  if (value < min)
  {
    return min;
  }
  if (value > max)
  {
    return max;
  }
  return value;
}

static uint32_t Reverse_Rc_Pwm(uint32_t input_us)
{
  input_us = Clamp_U32(input_us, RC_PWM_MIN_US, RC_PWM_MAX_US);
  return (RC_PWM_MIN_US + RC_PWM_MAX_US) - input_us;
}

static uint32_t Apply_Throttle_Input_Filter(uint32_t input_us)
{
  static uint32_t last_throttle_us = THROTTLE_CENTER_US;
  
  input_us = Clamp_U32(input_us, RC_PWM_MIN_US, RC_PWM_MAX_US);
  
  // 최소 변화량 미만이면 이전값 유지 (노이즈 필터)
  if ((input_us > last_throttle_us && (input_us - last_throttle_us) < THROTTLE_INPUT_FILTER_US) ||
      (last_throttle_us > input_us && (last_throttle_us - input_us) < THROTTLE_INPUT_FILTER_US))
  {
    return last_throttle_us;
  }
  
  last_throttle_us = input_us;
  return input_us;
}

static uint32_t Apply_Throttle_Curve(uint32_t input_us)
{
  int32_t input_delta;
  int32_t linear_delta;
  int32_t cubic_delta;
  int32_t output_delta;
  int64_t cubic_input;

  input_us = Clamp_U32(input_us, RC_PWM_MIN_US, RC_PWM_MAX_US);
  input_delta = (int32_t)input_us - RC_PWM_CENTER_US;

  if ((input_delta > -THROTTLE_DEADBAND_US) && (input_delta < THROTTLE_DEADBAND_US))
  {
    return THROTTLE_CENTER_US;
  }

  linear_delta = (input_delta * (THROTTLE_MAX_US - THROTTLE_CENTER_US)) /
                 (RC_PWM_MAX_US - RC_PWM_CENTER_US);

  cubic_input = (int64_t)input_delta * input_delta * input_delta;
  cubic_delta = (int32_t)((cubic_input * (THROTTLE_MAX_US - THROTTLE_CENTER_US)) /
                          ((int64_t)(RC_PWM_MAX_US - RC_PWM_CENTER_US) *
                           (RC_PWM_MAX_US - RC_PWM_CENTER_US) *
                           (RC_PWM_MAX_US - RC_PWM_CENTER_US)));

  output_delta = ((linear_delta * (100 - THROTTLE_EXPO_PERCENT)) +
                  (cubic_delta * THROTTLE_EXPO_PERCENT)) / 100;

  return Clamp_U32((uint32_t)(THROTTLE_CENTER_US + output_delta),
                   THROTTLE_MIN_US,
                   THROTTLE_MAX_US);
}

static uint32_t Apply_Throttle_Direction_Guard(uint32_t target_us)
{
  uint32_t now = HAL_GetTick();
  int32_t current_delta = (int32_t)throttle_output_us - THROTTLE_CENTER_US;
  int32_t target_delta = (int32_t)target_us - THROTTLE_CENTER_US;

  if ((current_delta > THROTTLE_DEADBAND_US && target_delta < -THROTTLE_DEADBAND_US) ||
      (current_delta < -THROTTLE_DEADBAND_US && target_delta > THROTTLE_DEADBAND_US))
  {
    throttle_neutral_hold_until = now + THROTTLE_REVERSE_HOLD_MS;
    throttle_output_us = THROTTLE_CENTER_US;
  }

  if ((int32_t)(throttle_neutral_hold_until - now) > 0)
  {
    return THROTTLE_CENTER_US;
  }

  return target_us;
}

static uint32_t Apply_Throttle_Slew(uint32_t target_us)
{
  if (target_us > throttle_output_us + THROTTLE_SLEW_STEP_US)
  {
    throttle_output_us += THROTTLE_SLEW_STEP_US;
  }
  else if (target_us + THROTTLE_SLEW_STEP_US < throttle_output_us)
  {
    throttle_output_us -= THROTTLE_SLEW_STEP_US;
  }
  else
  {
    throttle_output_us = target_us;
  }

  return throttle_output_us;
}

static uint32_t Apply_Steering_Jitter_Filter(uint32_t target_us)
{
  target_us = Clamp_U32(target_us, RC_PWM_MIN_US, RC_PWM_MAX_US);

  if ((target_us > steering_output_us && (target_us - steering_output_us) < STEERING_JITTER_DEADBAND_US) ||
      (steering_output_us > target_us && (steering_output_us - target_us) < STEERING_JITTER_DEADBAND_US))
  {
    return steering_output_us;
  }

  steering_output_us = target_us;
  return steering_output_us;
}

/**
 * @brief 3-position 스위치 입력에서 깜빡이 상태를 추출합니다.
 * @param switch_input_us: 스위치 입력값 (μs)
 * @retval uint8_t: 0=NONE, 1=RIGHT, 2=LEFT
 */
static uint8_t Get_Blink_State(uint32_t switch_input_us)
{
  if (switch_input_us < (SWITCH_CENTER_US - SWITCH_DEADBAND_US))
  {
    return 1;  // RIGHT
  }
  else if (switch_input_us > (SWITCH_CENTER_US + SWITCH_DEADBAND_US))
  {
    return 2;  // LEFT
  }
  else
  {
    return 0;  // NONE (CENTER)
  }
}

static int8_t Calculate_Virtual_Speed_Kmh(uint32_t throttle_us)
{
  int32_t delta;
  int32_t speed;

  throttle_us = Clamp_U32(throttle_us, THROTTLE_MIN_US, THROTTLE_MAX_US);
  delta = (int32_t)throttle_us - THROTTLE_CENTER_US;

  if ((delta > -THROTTLE_DEADBAND_US) && (delta < THROTTLE_DEADBAND_US))
  {
    return 0;
  }

  speed = (delta * VIRTUAL_SPEED_MAX_KMH) / (THROTTLE_MAX_US - THROTTLE_CENTER_US);
  if (speed > VIRTUAL_SPEED_MAX_KMH)
  {
    speed = VIRTUAL_SPEED_MAX_KMH;
  }
  else if (speed < -VIRTUAL_SPEED_MAX_KMH)
  {
    speed = -VIRTUAL_SPEED_MAX_KMH;
  }

  return (int8_t)speed;
}

static int8_t Calculate_Steering_Angle_Deg(uint32_t steering_us)
{
  int32_t delta;
  int32_t angle;

  steering_us = Clamp_U32(steering_us, RC_PWM_MIN_US, RC_PWM_MAX_US);
  delta = (int32_t)steering_us - RC_PWM_CENTER_US;

  if ((delta > -STEERING_JITTER_DEADBAND_US) && (delta < STEERING_JITTER_DEADBAND_US))
  {
    return 0;
  }

  angle = (delta * STEERING_MAX_DEG) / (RC_PWM_MAX_US - RC_PWM_CENTER_US);
  if (angle > STEERING_MAX_DEG)
  {
    angle = STEERING_MAX_DEG;
  }
  else if (angle < -STEERING_MAX_DEG)
  {
    angle = -STEERING_MAX_DEG;
  }

  return (int8_t)angle;
}

static uint16_t Get_U16_LE(const uint8_t *buf)
{
  return (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
}

static uint8_t SPI_Calculate_CRC8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0;

  for (uint8_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ SPI_CRC8_POLYNOMIAL)
                          : (uint8_t)(crc << 1);
    }
  }

  return crc;
}

static uint8_t SPI_Is_Valid_Frame(const uint8_t *frame)
{
  return frame[0] == SPI_SYNC_0 &&
         frame[1] == SPI_SYNC_1 &&
         frame[7] == SPI_Calculate_CRC8(frame, SPI_FRAME_LEN - 1);
}

static void SPI1_Consume_Rx_Byte(uint8_t value)
{
  if (spi_rx_frame_index == 0)
  {
    if (value == SPI_SYNC_0)
    {
      spi_rx_frame[0] = value;
      spi_rx_frame_index = 1;
    }
    return;
  }

  if (spi_rx_frame_index == 1)
  {
    if (value == SPI_SYNC_1)
    {
      spi_rx_frame[1] = value;
      spi_rx_frame_index = 2;
    }
    else if (value == SPI_SYNC_0)
    {
      spi_rx_frame[0] = value;
    }
    else
    {
      spi_rx_frame_index = 0;
    }
    return;
  }

  spi_rx_frame[spi_rx_frame_index++] = value;
  if (spi_rx_frame_index < SPI_FRAME_LEN)
  {
    return;
  }

  spi_rx_frame_index = 0;
  if (!SPI_Is_Valid_Frame(spi_rx_frame))
  {
    spi_invalid_frame_count++;
    return;
  }

  // MOSI: AA 55 | seq | TTC LSB | TTC MSB | flags | reserved | CRC8
  spi_last_rx_sequence = spi_rx_frame[2];
  received_ttc_x100 = Get_U16_LE(&spi_rx_frame[3]);
  rpi_flags = spi_rx_frame[5];
  lane_change_allowed = (rpi_flags & 0x01);
  spi_ok_count++;
}

static void SPI1_Prepare_Tx_Data(void)
{
  uint32_t steering_us = IC_Val2;
  int8_t speed_kmh = Calculate_Virtual_Speed_Kmh(throttle_output_us);
  int8_t steering_deg;
  uint8_t current_blink_state = Get_Blink_State(IC_Val3);

  /*
   * SPI 조향값은 TIM3에서 실제 캡처한 RC 입력을 기준으로 만든다.
   * 아직 캡처 전이거나 신호가 유효 범위를 벗어나면 마지막 출력값을 사용한다.
   */
  if (steering_us >= RC_PWM_MIN_US && steering_us <= RC_PWM_MAX_US)
  {
    steering_us = Reverse_Rc_Pwm(steering_us);
  }
  else
  {
    steering_us = steering_output_us;
  }
  steering_deg = Calculate_Steering_Angle_Deg(steering_us);

  // MISO: AA 55 | seq | speed | steering | blink | status | CRC8
  spi_tx_buffer[0] = SPI_SYNC_0;
  spi_tx_buffer[1] = SPI_SYNC_1;
  spi_tx_buffer[2] = spi_tx_sequence++;
  spi_tx_buffer[3] = (uint8_t)speed_kmh;
  spi_tx_buffer[4] = (uint8_t)steering_deg;
  spi_tx_buffer[5] = (current_blink_state & 0x03);
  spi_tx_buffer[6] = (lane_change_allowed & 0x01);
  spi_tx_buffer[7] = SPI_Calculate_CRC8(spi_tx_buffer, SPI_FRAME_LEN - 1);

  virtual_speed_kmh = speed_kmh;
  steering_angle_deg = steering_deg;
  blink_state = current_blink_state;
  blink_state_prev = current_blink_state;
}

static HAL_StatusTypeDef SPI1_Arm_Transfer(void)
{
  HAL_StatusTypeDef status;

  SPI1_Prepare_Tx_Data();
  status = HAL_SPI_TransmitReceive_IT(
      &hspi1,
      spi_tx_buffer,
      spi_rx_buffer,
      SPI_FRAME_LEN);
  spi_last_status = status;
  spi_transfer_armed = (status == HAL_OK);

  if (status != HAL_OK)
  {
    spi_error_count++;
  }

  return status;
}

static void SPI1_Process_Completed_Transfer(void)
{
  for (uint8_t i = 0; i < SPI_FRAME_LEN; i++)
  {
    spi_last_tx_buffer[i] = spi_tx_buffer[i];
    spi_last_rx_buffer[i] = spi_rx_buffer[i];
  }

  for (uint8_t i = 0; i < SPI_FRAME_LEN; i++)
  {
    SPI1_Consume_Rx_Byte(spi_rx_buffer[i]);
  }

  spi_last_tx_tick = HAL_GetTick();
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_LPUART1_UART_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // TIM2 PWM Input 시작 (CH1은 주기용, CH2는 펄스폭용으로 자동 할당됨)
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  // TIM3 PWM Input 시작
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

  // TIM4 3-position switch input 시작
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

  // TIM1 PWM Output 시작
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // CH1 - 구동모터
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // CH2 - 서보모터
  
  motor_start_tick = HAL_GetTick();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_ARM_US);

  if (SPI1_Arm_Transfer() != HAL_OK)
  {
    printf("[SPI ERROR] Initial arm failed: %d\r\n", spi_last_status);
  }

  printf("RC Vehicle Control Start!\r\n");
  printf("TIM2(Throttle) -> TIM1 CH1 | TIM3(Servo) -> TIM1 CH2 | TIM4(Switch)\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    // 오류 콜백 이후 재-arm이 실패한 경우 메인 루프에서 복구한다.
    if (!spi_transfer_armed && hspi1.State == HAL_SPI_STATE_READY)
    {
      SPI1_Arm_Transfer();
    }

    // 비블로킹 디버그 출력 (100ms 주기)
    static uint32_t debug_last_tick = 0;
    if (now - debug_last_tick > 100)
    {
      const char *blink_str = (blink_state == 1) ? "RIGHT" : 
                              (blink_state == 2) ? "LEFT" : "NONE";
      GPIO_PinState spi_nss_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
      printf("Throttle IN: %lu -> OUT: %lu | Servo IN: %lu -> OUT: %lu | "
             "Speed:%d km/h Steer:%d deg | Blink: %s(%u) | TTC:%u.%02us LaneOK:%u | "
             "SPI TX:%02X%02X S:%02X D:%02X/%02X/%02X C:%02X | "
             "RX:%02X%02X S:%02X D:%02X/%02X/%02X C:%02X | "
             "ST:%d OK:%lu INV:%lu ERR:%lu NSS:%u Age:%lums\r\n",
             IC_Val1, throttle_output_us, IC_Val2, steering_output_us, 
             (int)virtual_speed_kmh, (int)steering_angle_deg,
             blink_str, blink_state,
             (unsigned int)(received_ttc_x100 / 100U),
             (unsigned int)(received_ttc_x100 % 100U),
             lane_change_allowed,
             spi_last_tx_buffer[0], spi_last_tx_buffer[1], spi_last_tx_buffer[2],
             spi_last_tx_buffer[3], spi_last_tx_buffer[4], spi_last_tx_buffer[5],
             spi_last_tx_buffer[7],
             spi_last_rx_buffer[0], spi_last_rx_buffer[1], spi_last_rx_buffer[2],
             spi_last_rx_buffer[3], spi_last_rx_buffer[4], spi_last_rx_buffer[5],
             spi_last_rx_buffer[7],
             spi_last_status, spi_ok_count, spi_invalid_frame_count, spi_error_count,
             spi_nss_state, now - spi_last_tx_tick);
      debug_last_tick = now;
    }

    HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  // 비블로킹 Slave 송수신 완료/오류 콜백을 위한 SPI1 IRQ 활성화
  HAL_NVIC_SetPriority(SPI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  sConfigOC.Pulse = 1500;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * CubeMX가 stm32g4xx_it.c에 strong SPI1_IRQHandler를 생성한 경우 그
 * 핸들러가 우선한다. 생성되지 않은 프로젝트에서도 동작하도록 weak
 * fallback을 둔다.
 */
__weak void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
    {
        return;
    }

    spi_transfer_armed = 0;
    spi_last_status = HAL_OK;
    SPI1_Process_Completed_Transfer();

    // 다음 Master transaction 전에 즉시 새 프레임을 TX 레지스터에 준비한다.
    SPI1_Arm_Transfer();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
    {
        return;
    }

    spi_transfer_armed = 0;
    spi_last_status = HAL_ERROR;
    spi_error_count++;
    received_ttc_x100 = TTC_UNAVAILABLE_X100;
    rpi_flags = 0;
    lane_change_allowed = 0;

    HAL_SPI_Abort_IT(hspi);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        SPI1_Arm_Transfer();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // 펄스폭(High 구간) 측정이 완료되는 시점인 'CH2 인터럽트'에서만 동작
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        /* ===================================================================
         * [구동 모터 제어 필터 체인 - TIM2]
         * =================================================================== */
        if (htim->Instance == TIM2)
        {
            uint32_t throttle_target_us;
            uint32_t final_slew_output;

            // 쓰로틀 입력 (TIM2) 펄스폭 읽기
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            /* [알고리즘 1. 초기 기동 가드 (Motor Arming Process)] */
            if ((HAL_GetTick() - motor_start_tick) < MOTOR_ARM_TIME_MS)
            {
                throttle_output_us = THROTTLE_CENTER_US;
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_ARM_US);
                return;
            }

            /* [필터 체인 실행] */
            // 0. 입력 원시 노이즈 데드밴드 프리필터
            throttle_target_us = Apply_Throttle_Input_Filter(IC_Val1);
            
            // 1. [알고리즘 3. 데드밴드] & [알고리즘 2. 대역 스케일링] & [알고리즘 4. 지수 커브 필터] 공통 처리
            throttle_target_us = Apply_Throttle_Curve(throttle_target_us);
            
            // 2. [알고리즘 5. 역기전력 방지 방향 가드 (Direction Guard)]
            throttle_target_us = Apply_Throttle_Direction_Guard(throttle_target_us);

            // 3. 변속기 하드웨어 추종 충격 완화를 위한 Slew Rate 필터 적용
            final_slew_output = Apply_Throttle_Slew(throttle_target_us);

            /* [안전성 보완] 최종 변속기 출력 직전 하드웨어 안전 출력 범위(1300~1700) 완전 락(Lock) */
            final_slew_output = Clamp_U32(final_slew_output, THROTTLE_MIN_US, THROTTLE_MAX_US);

            // TIM1_CH1(구동모터)로 최종 필터링된 안전 신호 인가
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, final_slew_output);
        }
        
        /* ===================================================================
         * [조향 서보 제어 필터 체인 - TIM3]
         * =================================================================== */
        else if (htim->Instance == TIM3)
        {
            uint32_t steering_target_us;

            // 조향 입력 (TIM3) 펄스폭 읽기
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            // 하드웨어 특성에 따른 반전 매핑 처리
            steering_target_us = Reverse_Rc_Pwm(IC_Val2);
            
            /* [알고리즘 6. 조향 지터 및 서보 보호 필터 (Steering Jitter Filter)] */
            steering_target_us = Apply_Steering_Jitter_Filter(steering_target_us);

            // TIM1_CH2(서보모터)로 최종 조향 필터링 값 출력
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, steering_target_us);
        }
        
        /* ===================================================================
         * [보조 스위치 - TIM4]
         * =================================================================== */
        else if (htim->Instance == TIM4)
        {
            // 3-position 스위치 입력 (TIM4) 펄스폭 읽기
            IC_Val3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
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
