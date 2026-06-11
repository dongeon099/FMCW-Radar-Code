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
#pragma pack(push, 1)  // 1바이트 정렬로 패딩 공간을 완전히 제거 (총 33바이트)
typedef struct {
    uint8_t header1;        // 0xAA (인덱스 0)
    uint8_t header2;        // 0x55 (인덱스 1)
    uint8_t version;        // 0x01 (인덱스 2)
    uint8_t seq;            // 프레임 카운터 (인덱스 3)
    uint8_t turn_signal;    // RPi가 인지한 깜빡이 (인덱스 4)
    uint8_t object_count;   // 추적 객체 수 (인덱스 5)
    uint8_t risk_level;     // ★ 박 님이 가져오실 핵심 위험도! (인덱스 6)
    uint8_t lane_id;        // 타겟 차선 정보 (인덱스 7)
    uint16_t rec_speed;     // 추천 속도 (인덱스 8~9)
    uint16_t safe_dist;     // 안전 거리 (인덱스 10~11)
    uint16_t ttc;           // 충돌 예상 시간 (인덱스 12~13)
    uint8_t reserved[18];   // 예약 영역 (인덱스 14~31)
    uint8_t checksum;       // 체크섬 (인덱스 32)
} RPi_MOSI_Packet_t;

typedef struct {
    uint8_t header1;        // 0xAA
    uint8_t header2;        // 0x55
    uint8_t version;        // 0x01
    int16_t ego_speed;      // ego_speed * 100 (km/h * 100 = m/s * 36 * 100, little-endian)
    int16_t steering_angle; // steering_angle * 10 (degrees * 10, little-endian)
    uint8_t turn_signal;    // 0=NONE, 1=RIGHT, 2=LEFT (인덱스 7)
    uint8_t reserved[24];   // 예약 (인덱스 8~31)
    uint8_t checksum;       // 체크섬 (인덱스 32)
} STM32_MISO_Packet_t;
#pragma pack(pop)
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

/* [가상 속도 계산] RC 카 사양: 3kg, 25T 모터, TT02 섀시 */
/* 최대 속도: 50 km/h (단위: km/h * 100) */
#define VIRTUAL_SPEED_MAX_KMH_X100  5000  // 50.00 km/h
/* 쓰로틀 범위: 1300~1700 µs (중심 1500 µs) */
#define THROTTLE_SPEED_RANGE_US     200   // (1700-1500) = 200 µs

/* [가상 조향각도] Hitec HS422 서보모터 */
/* HS422: ±45도 스트로크, 1000-2000µs PWM 범위 */
#define SERVO_STEERING_MIN_DEG      -45   // 좌회전 최대 (-45도)
#define SERVO_STEERING_MAX_DEG      45    // 우회전 최대 (+45도)
#define SERVO_PWM_MIN_US            1000  // 좌회전 최대 PWM
#define SERVO_PWM_CENTER_US         1500  // 중립 PWM
#define SERVO_PWM_MAX_US            2000  // 우회전 최대 PWM
#define SERVO_STEERING_DEADBAND_US  10    // 조향 데드밴드 (±10µs)

/* [SPI 패킷 크기] 33바이트 고정 */
#define SPI_PACKET_LEN 33
#define HEADER1 0xAA
#define HEADER2 0x55
#define VERSION 0x01

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

// SPI1 송수신 버퍼 (33바이트 패킷)
static uint8_t spi_tx_buffer[SPI_PACKET_LEN] = {0};
static uint8_t spi_rx_buffer[SPI_PACKET_LEN] = {0};
static volatile uint8_t spi_last_tx_buffer[SPI_PACKET_LEN] = {0};
static volatile uint8_t spi_last_rx_buffer[SPI_PACKET_LEN] = {0};
static volatile HAL_StatusTypeDef spi_last_status = HAL_OK;
static volatile uint32_t spi_ok_count = 0;
static volatile uint32_t spi_error_count = 0;
static volatile uint8_t spi_rx_ready = 0;
static volatile uint32_t spi_hal_error_code = HAL_SPI_ERROR_NONE;

// 깜빡이 상태 추적
static volatile uint8_t blink_state = 0;  // 0=NONE, 1=RIGHT, 2=LEFT
static volatile uint8_t blink_state_prev = 0;  // 상태 변경 감지용
static volatile uint8_t lane_change_allowed = 0;  // 라즈베리파이 응답

// SPI 타이밍
static uint32_t spi_last_tx_tick = 0;

// 라즈베리파이로부터 수신한 TTC 값
static volatile uint16_t received_ttc_x100 = 0xFFFF;  // TTC 미사용 상태

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

volatile int16_t g_ego_speed_X100 = 0;  // 현재 차량의 가상 속도 (km/h * 100)
volatile int16_t g_steer_angle_X10 = 0; // 현재 차량의 조향각 (deg * 10)

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
static HAL_StatusTypeDef SPI1_Exchange_Data(uint8_t blink_state, int16_t ego_speed_x100, int16_t steering_angle_x10);
static void SPI1_Build_Tx_Packet(uint8_t blink_state, int16_t ego_speed_x100, int16_t steering_angle_x10);
static void SPI1_Process_Received_Data(void);

/* 가상 속도/조향각도 계산 함수 */
static int16_t Calculate_Virtual_Speed(uint32_t throttle_output_us);
static int16_t Calculate_Virtual_Steering_Angle(uint32_t servo_input_us);
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

/**
 * @brief PWM 출력값에 따른 가상 속도 값 계산
 * @param throttle_output_us: 쓰로틀 PWM 출력값 (µs)
 * @retval int16_t: 속도 (km/h * 100, 음수는 역방향)
 * 
 * 선형 맵핑:
 * - 1300 µs (THROTTLE_MIN_US): -50 km/h
 * - 1500 µs (THROTTLE_CENTER_US): 0 km/h
 * - 1700 µs (THROTTLE_MAX_US): +50 km/h
 */
static int16_t Calculate_Virtual_Speed(uint32_t throttle_output_us)
{
  int32_t throttle_delta;
  int32_t speed_X100;

  /* 쓰로틀 출력값을 중립을 기준으로 한 델타값으로 변환 */
  throttle_delta = (int32_t)throttle_output_us - THROTTLE_CENTER_US;

  /* 데드밴드 처리 (±8µs 이내면 0 속도) */
  if (throttle_delta > -THROTTLE_DEADBAND_US && throttle_delta < THROTTLE_DEADBAND_US)
  {
    return 0;
  }

  /* 선형 맵핑: throttle_delta (±200µs) -> speed (±50km/h = ±5000 km/h*100) */
  /* 계산식: speed = (throttle_delta / THROTTLE_SPEED_RANGE_US) * VIRTUAL_SPEED_MAX_KMH_X100 */
  /* = (throttle_delta / 200) * 5000 = throttle_delta * 25 */
  speed_X100 = (throttle_delta * VIRTUAL_SPEED_MAX_KMH_X100) / THROTTLE_SPEED_RANGE_US;

  /* 범위 제한 */
  if (speed_X100 > VIRTUAL_SPEED_MAX_KMH_X100)
  {
    speed_X100 = VIRTUAL_SPEED_MAX_KMH_X100;
  }
  else if (speed_X100 < -VIRTUAL_SPEED_MAX_KMH_X100)
  {
    speed_X100 = -VIRTUAL_SPEED_MAX_KMH_X100;
  }

  return (int16_t)speed_X100;
}

/**
 * @brief 서보모터 PWM 입력값에 따른 가상 조향각도 계산
 * @param servo_input_us: 서보모터 PWM 입력값 (µs)
 * @retval int16_t: 조향각도 (degree * 10)
 * 
 * Hitec HS422 서보모터 사양:
 * - 범위: ±45도 (총 90도 스트로크)
 * - PWM 범위: 1000~2000 µs
 * - 중립: 1500 µs
 * 
 * 선형 맵핑:
 * - 1000 µs (SERVO_PWM_MIN_US): -45도 (좌회전 최대)
 * - 1500 µs (SERVO_PWM_CENTER_US): 0도 (중립)
 * - 2000 µs (SERVO_PWM_MAX_US): +45도 (우회전 최대)
 */
static int16_t Calculate_Virtual_Steering_Angle(uint32_t servo_input_us)
{
  int32_t servo_delta;
  int32_t angle_deg;

  /* 서보 입력값을 중립을 기준으로 한 델타값으로 변환 */
  servo_delta = (int32_t)servo_input_us - SERVO_PWM_CENTER_US;

  /* 데드밴드 처리 (±10µs 이내면 0도) */
  if (servo_delta > -SERVO_STEERING_DEADBAND_US && servo_delta < SERVO_STEERING_DEADBAND_US)
  {
    return 0;
  }

  /* 선형 맵핑: servo_delta (±500µs) -> angle (±45도)
   * 계산식: angle = (servo_delta / 500) * 45 = servo_delta * 0.09 = servo_delta * 9 / 100
   * 하지만 degree * 10 단위로 저장하므로: angle_x10 = servo_delta * 90 / 500 = servo_delta * 9 / 50
   */
  angle_deg = (servo_delta * (SERVO_STEERING_MAX_DEG - 0) * 10) / (SERVO_PWM_MAX_US - SERVO_PWM_CENTER_US);

  /* 범위 제한 */
  if (angle_deg > (SERVO_STEERING_MAX_DEG * 10))
  {
    angle_deg = SERVO_STEERING_MAX_DEG * 10;
  }
  else if (angle_deg < (SERVO_STEERING_MIN_DEG * 10))
  {
    angle_deg = SERVO_STEERING_MIN_DEG * 10;
  }

  return (int16_t)angle_deg;
}

/**
 * @brief SPI1 양방향 송수신 (Full Duplex Slave, 33바이트)
 * @param blink_state: 깜빡이 상태 (0=NONE, 1=RIGHT, 2=LEFT)
 * @param ego_speed_x100: 가상 속도 (km/h * 100, int16)
 * @param steering_angle_x10: 조향각 (deg * 10, int16)
 * @retval HAL_StatusTypeDef: HAL_OK (성공) 또는 에러 코드
 * 
 * MISO 패킷 (STM32 → Rpi):
 *   [0]    0xAA (header1)
 *   [1]    0x55 (header2)
 *   [2]    0x01 (version)
 *   [3]    ego_speed LSB
 *   [4]    ego_speed MSB (int16)
 *   [5]    steering_angle LSB
 *   [6]    steering_angle MSB (int16)
 *   [7]    turn_signal (0=NONE, 1=RIGHT, 2=LEFT)
 *   [8-31] reserved
 *   [32]   checksum
 * 
 * MOSI 패킷 (Rpi → STM32):
 *   [0-7]  header + control fields
 *   [8-9]  recommended_speed (x100 m/s)
 *   [10-11] safe_distance (x100 m)
 *   [12-13] TTC (x100 s) ← 핵심!
 *   [14-31] reserved
 *   [32]   checksum
 */
static uint8_t compute_checksum(const uint8_t *packet, int len)
{
  uint32_t sum = 0;
  for (int i = 0; i < len - 1; i++)  // 마지막 바이트(체크섬)는 제외
  {
    sum += packet[i];
  }
  return (uint8_t)(sum & 0xFF);
}

static void SPI1_Build_Tx_Packet(
    uint8_t current_blink_state,
    int16_t ego_speed_x100,
    int16_t steering_angle_x10)
{
  // TX 버퍼 구성 (MISO 패킷)
  spi_tx_buffer[0] = HEADER1;
  spi_tx_buffer[1] = HEADER2;
  spi_tx_buffer[2] = VERSION;
  spi_tx_buffer[3] = (uint8_t)(ego_speed_x100 & 0xFF);         // LSB
  spi_tx_buffer[4] = (uint8_t)((ego_speed_x100 >> 8) & 0xFF);  // MSB
  spi_tx_buffer[5] = (uint8_t)(steering_angle_x10 & 0xFF);     // LSB
  spi_tx_buffer[6] = (uint8_t)((steering_angle_x10 >> 8) & 0xFF); // MSB
  spi_tx_buffer[7] = current_blink_state & 0x03;
  for (int i = 8; i < SPI_PACKET_LEN - 1; i++)
  {
    spi_tx_buffer[i] = 0;  // reserved 영역 초기화
  }
  spi_tx_buffer[SPI_PACKET_LEN - 1] = compute_checksum(spi_tx_buffer, SPI_PACKET_LEN);
}

static HAL_StatusTypeDef SPI1_Exchange_Data(
    uint8_t current_blink_state,
    int16_t ego_speed_x100,
    int16_t steering_angle_x10)
{
  SPI1_Build_Tx_Packet(current_blink_state, ego_speed_x100, steering_angle_x10);

  // 슬레이브를 상시 수신 대기 상태로 둔다. 완료 콜백에서 즉시 다시 등록한다.
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(
      &hspi1, spi_tx_buffer, spi_rx_buffer, SPI_PACKET_LEN);
  spi_last_status = status;

  if (status != HAL_OK)
  {
    spi_error_count++;
    spi_hal_error_code = HAL_SPI_GetError(&hspi1);
  }

  return status;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI1)
  {
    return;
  }

  for (int i = 0; i < SPI_PACKET_LEN; i++)
  {
    spi_last_tx_buffer[i] = spi_tx_buffer[i];
    spi_last_rx_buffer[i] = spi_rx_buffer[i];
  }
  spi_last_status = HAL_OK;
  spi_ok_count++;
  spi_last_tx_tick = HAL_GetTick();
  spi_rx_ready = 1;

  // 다음 마스터 전송 전에 최신 차량 상태를 MISO 버퍼에 준비한다.
  SPI1_Build_Tx_Packet(blink_state, g_ego_speed_X100, g_steer_angle_X10);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(
      &hspi1, spi_tx_buffer, spi_rx_buffer, SPI_PACKET_LEN);
  if (status != HAL_OK)
  {
    spi_last_status = status;
    spi_hal_error_code = HAL_SPI_GetError(&hspi1);
    spi_error_count++;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI1)
  {
    return;
  }

  spi_last_status = HAL_ERROR;
  spi_hal_error_code = HAL_SPI_GetError(hspi);
  spi_error_count++;

  SPI1_Build_Tx_Packet(blink_state, g_ego_speed_X100, g_steer_angle_X10);
  if (HAL_SPI_TransmitReceive_IT(
          &hspi1, spi_tx_buffer, spi_rx_buffer, SPI_PACKET_LEN) != HAL_OK)
  {
    spi_last_status = HAL_ERROR;
  }
}

void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi1);
}

static void SPI1_Process_Received_Data(void)
{
  uint8_t rx[SPI_PACKET_LEN];

  __disable_irq();
  if (!spi_rx_ready)
  {
    __enable_irq();
    return;
  }
  for (int i = 0; i < SPI_PACKET_LEN; i++)
  {
    rx[i] = spi_last_rx_buffer[i];
  }
  spi_rx_ready = 0;
  __enable_irq();

  if (rx[0] != HEADER1 || rx[1] != HEADER2 || rx[2] != VERSION)
  {
    spi_error_count++;
    lane_change_allowed = 0;
    printf("[SPI ERR-HEADER] [0-7]=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
    return;
  }

  uint8_t calculated_checksum = compute_checksum(rx, SPI_PACKET_LEN);
  if (calculated_checksum != rx[SPI_PACKET_LEN - 1])
  {
    spi_error_count++;
    lane_change_allowed = 0;
    printf("[SPI ERR-CHKSUM] [0-7]=%02X %02X %02X %02X %02X %02X %02X %02X | calc=%02X recv=%02X\r\n",
           rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7],
           calculated_checksum, rx[SPI_PACKET_LEN - 1]);
    return;
  }

  received_ttc_x100 = rx[12] | ((uint16_t)rx[13] << 8);
  lane_change_allowed = 1;
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

  // RPi가 언제 전송하더라도 받을 수 있도록 부팅 직후 SPI 슬레이브를 대기시킨다.
  if (SPI1_Exchange_Data(blink_state, g_ego_speed_X100, g_steer_angle_X10) != HAL_OK)
  {
    printf("[SPI START ERROR] HAL=%d ERR=0x%08lX\r\n",
           spi_last_status, spi_hal_error_code);
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

    // SPI 완료 콜백이 복사해 둔 MOSI 패킷을 메인 루프에서 처리한다.
    SPI1_Process_Received_Data();

    // MISO에 실을 차량 상태를 50ms 주기로 갱신한다.
    static uint32_t spi_tick = 0;
    uint8_t current_blink_state = Get_Blink_State(IC_Val3);
    
    if ((now - spi_tick > 50) || (current_blink_state != blink_state_prev))
    {
      // 가상 속도 계산 (PWM 출력값 기반)
      g_ego_speed_X100 = Calculate_Virtual_Speed(throttle_output_us);
      
      // 가상 조향각도 계산 (Hitec HS422 기반)
      g_steer_angle_X10 = Calculate_Virtual_Steering_Angle(IC_Val2);

      blink_state = current_blink_state;
      blink_state_prev = current_blink_state;
      spi_tick = now;
    }

    // 비블로킹 디버그 출력 (100ms 주기)
    static uint32_t debug_last_tick = 0;
    if (now - debug_last_tick > 100)
    {
      const char *blink_str = (blink_state == 1) ? "RIGHT" : 
                              (blink_state == 2) ? "LEFT" : "NONE";
      
      // TTC 값 표시 (0xFFFF는 미사용)
      const char *ttc_str = (received_ttc_x100 == 0xFFFF) ? "N/A" : "";
      
      printf("Speed: %d.%02d km/h | Angle: %d.%d° | Blink: %s | "
             "TTC: %s%u.%02u s | OK:%lu ERR:%lu HAL:%d/0x%08lX Age:%lums\r\n",
             g_ego_speed_X100 / 100, g_ego_speed_X100 % 100,
             g_steer_angle_X10 / 10, g_steer_angle_X10 % 10,
             blink_str,
             ttc_str,
             (received_ttc_x100 != 0xFFFF) ? (received_ttc_x100 / 100) : 0,
             (received_ttc_x100 != 0xFFFF) ? (received_ttc_x100 % 100) : 0,
             spi_ok_count, spi_error_count,
             spi_last_status, spi_hal_error_code,
             now - spi_last_tx_tick);
      debug_last_tick = now;
    }
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
  HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
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
