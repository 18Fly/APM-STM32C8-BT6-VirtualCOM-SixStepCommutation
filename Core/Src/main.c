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
 * This software is licensed under terms that can be found in the LICENSE
 file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes                                                                    \
------------------------------------------------------------------*/           \
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "usb_device.h"

/* Private includes
----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef
-----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
  float kp, ki, kd;
  float integral;
  float prev_error;
  uint32_t last_ms;
} PID_t;

/* USER CODE END PTD */

/* Private define
------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 电机极对数: 转子上永磁体总数量 / 2
#define MOTOR_POLE_PAIRS 7

// 定义音符频率 (Hz)
#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978
#define REST 0

/* USER CODE END PD */

/* Private macro
-------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables
---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// ADC 采样值以及中间值
volatile uint32_t theFirstAdcU;
volatile uint32_t theSecondAdcU;
volatile uint32_t currentAdcU;

volatile uint32_t theFirstAdcV;
volatile uint32_t theSecondAdcV;
volatile uint32_t currentAdcV;

volatile uint32_t theFirstAdcW;
volatile uint32_t theSecondAdcW;
volatile uint32_t currentAdcW;

volatile uint32_t adcU;
volatile uint32_t adcV;
volatile uint32_t adcW;

// 乐谱结构体
typedef struct {
  uint16_t note;
  uint16_t duration;
} Tone;

// 超级马里奥 (Super Mario Bros Theme)
const Tone Mario_Theme[] = {
    //  Intro
    {NOTE_E5, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_C5, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_G5, 100},
    {REST, 300},
    {NOTE_G4, 100},
    {REST, 300},

    //  Theme A (Main)
    {NOTE_C5, 100},
    {REST, 200},
    {NOTE_G4, 100},
    {REST, 200},
    {NOTE_E4, 100},
    {REST, 200},
    {NOTE_A4, 100},
    {REST, 100},
    {NOTE_B4, 100},
    {REST, 100},
    {NOTE_AS4, 100},
    {NOTE_A4, 100},

    {NOTE_G4, 66},
    {NOTE_E5, 66},
    {NOTE_G5, 66},
    {NOTE_A5, 100},
    {NOTE_F5, 100},
    {NOTE_G5, 100},
    {REST, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_C5, 100},
    {NOTE_D5, 100},
    {NOTE_B4, 100},
    {REST, 200},

    // Repeat Theme A
    {NOTE_C5, 100},
    {REST, 200},
    {NOTE_G4, 100},
    {REST, 200},
    {NOTE_E4, 100},
    {REST, 200},
    {NOTE_A4, 100},
    {REST, 100},
    {NOTE_B4, 100},
    {REST, 100},
    {NOTE_AS4, 100},
    {NOTE_A4, 100},

    {NOTE_G4, 66},
    {NOTE_E5, 66},
    {NOTE_G5, 66},
    {NOTE_A5, 100},
    {NOTE_F5, 100},
    {NOTE_G5, 100},
    {REST, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_C5, 100},
    {NOTE_D5, 100},
    {NOTE_B4, 100},
    {REST, 200},

    //  Theme B (Underground / Bridge section)
    {REST, 200},
    {NOTE_G5, 50},
    {NOTE_FS5, 50},
    {NOTE_F5, 50},
    {NOTE_DS5, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_GS4, 50},
    {NOTE_A4, 50},
    {NOTE_C5, 50},
    {REST, 50},
    {NOTE_A4, 50},
    {NOTE_C5, 50},
    {NOTE_D5, 100},

    {REST, 200},
    {NOTE_G5, 50},
    {NOTE_FS5, 50},
    {NOTE_F5, 50},
    {NOTE_DS5, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_C6, 100},
    {REST, 50},
    {NOTE_C6, 100},
    {NOTE_C6, 100},

    {REST, 200},
    {NOTE_G5, 50},
    {NOTE_FS5, 50},
    {NOTE_F5, 50},
    {NOTE_DS5, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_GS4, 50},
    {NOTE_A4, 50},
    {NOTE_C5, 50},
    {REST, 50},
    {NOTE_A4, 50},
    {NOTE_C5, 50},
    {NOTE_D5, 100},

    {REST, 100},
    {NOTE_DS5, 200},
    {REST, 100},
    {NOTE_D5, 200},
    {REST, 100},
    {NOTE_C5, 200},
    {REST, 400},

    //  Theme A (Return)
    {NOTE_C5, 100},
    {REST, 200},
    {NOTE_G4, 100},
    {REST, 200},
    {NOTE_E4, 100},
    {REST, 200},
    {NOTE_A4, 100},
    {REST, 100},
    {NOTE_B4, 100},
    {REST, 100},
    {NOTE_AS4, 100},
    {NOTE_A4, 100},

    {NOTE_G4, 66},
    {NOTE_E5, 66},
    {NOTE_G5, 66},
    {NOTE_A5, 100},
    {NOTE_F5, 100},
    {NOTE_G5, 100},
    {REST, 100},
    {NOTE_E5, 100},
    {REST, 100},
    {NOTE_C5, 100},
    {NOTE_D5, 100},
    {NOTE_B4, 100},
    {REST, 200},

    //  Ending
    {NOTE_C5, 100},
    {NOTE_C5, 100},
    {REST, 100},
    {NOTE_C5, 100},
    {REST, 100},
    {NOTE_C5, 100},
    {NOTE_D5, 100},
    {REST, 100},
    {NOTE_E5, 100},
    {NOTE_C5, 100},
    {REST, 100},
    {NOTE_A4, 100},
    {NOTE_G4, 100},
    {REST, 400},

    {NOTE_C5, 100},
    {NOTE_C5, 100},
    {REST, 100},
    {NOTE_C5, 100},
    {REST, 100},
    {NOTE_C5, 100},
    {NOTE_D5, 100},
    {NOTE_E5, 100},
    {REST, 400},

    // Game Over / Level Clear riff
    {NOTE_C5, 100},
    {REST, 50},
    {NOTE_G4, 100},
    {REST, 50},
    {NOTE_E4, 100},
    {NOTE_A4, 100},
    {NOTE_B4, 100},
    {NOTE_A4, 100},
    {NOTE_GS4, 100},
    {NOTE_AS4, 100},
    {NOTE_GS4, 100},
    {NOTE_G4, 100},
    {NOTE_F4, 100},
    {NOTE_G4, 400}};

// 星球大战 - 帝国进行曲 (Imperial March)
const Tone Imperial_March[] = {{NOTE_A4, 500}, {NOTE_A4, 500}, {NOTE_A4, 500},
                               {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 500},
                               {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 650},

                               {REST, 150},

                               {NOTE_E5, 500}, {NOTE_E5, 500}, {NOTE_E5, 500},
                               {NOTE_F5, 350}, {NOTE_C5, 150}, {NOTE_GS4, 500},
                               {NOTE_F4, 350}, {NOTE_C5, 150}, {NOTE_A4, 650},

                               {REST, 150}};

// 开环启动参数
uint32_t step_delay = 2000; // 初始换相延时 (us) - 越小转越快
// 闭环控制参数
volatile uint8_t run_mode = 0;               // 0: 开环启动, 1: 闭环运
volatile uint32_t last_commutation_time = 0; // 上一次换相时刻
volatile uint32_t period_time = 0;           // 换相周期 (60度电角度时
uint32_t closed_loop_threshold = 800;        // 闭环切换阈值
PID_t speed_pid = {
    .kp = 0.1f,   // 0.005
    .ki = 0.056f, // 0.0004
    .kd = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
};
// 目标转速 & 占空比限制
volatile uint32_t rpm = 0;            // 当前转速 (RPM)
volatile uint32_t target_rpm = 12000; // 目标转速 (RPM)
const uint32_t pwm_min = 300;
const uint32_t pwm_max = 3100;
// 过渡控制
volatile uint8_t pid_enabled = 0; // 闭环切入后延时使能PID
volatile uint32_t pid_enable_delay_ms = 50;

/* USER CODE END PV */

/* Private function prototypes
   -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code
---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 当前换相步骤 (0-5)
volatile uint8_t step = 0;
// 期望的 PWM 占空比
volatile uint32_t pwmDuty = 200;

/**
 * @brief 六步换相核心函数
 * @param step: 0-5 对应六个扇区
 * @param duty: PWM 比较值 (0 ~ Period)
 */
void SixStep_Commutate(uint8_t step, uint32_t duty) {
  // 预先定义 CCER 掩码，保留 CC4E (ADC触发)
  // 这里使用了互补输出 (CHxN)，所以同时使能 CCxE 和 CCxNE
  // 如果只想开单侧，请根据实际情况修改
  uint32_t ccer_mask = TIM_CCER_CC4E;

  switch (step) {
  case 0:
    // Step 1: U+ V- (W Float)
    TIM1->CCR1 = duty; // U: PWM
    TIM1->CCR2 = 0;
    // V: Low Side ON (CCR=0 -> CH2=0, CH2N=1)
    TIM1->CCR3 = 0;
    // W: Float (CCR无所谓，因为输出被禁用了)
    ccer_mask |=
        (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE);
    break;

  case 1: // Step 2: U+ W- (V Float)
    TIM1->CCR1 = duty;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0; // W: Low Side ON
    ccer_mask |=
        (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
    break;

  case 2: // Step 3: V+ W- (U Float)
    TIM1->CCR1 = 0;
    TIM1->CCR2 = duty;
    TIM1->CCR3 = 0;
    ccer_mask |=
        (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
    break;

  case 3:
    // Step 4: V+ U- (W Float)
    TIM1->CCR1 = 0; // U: Low Side ON
    TIM1->CCR2 = duty;
    TIM1->CCR3 = 0;
    ccer_mask |=
        (TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC1E | TIM_CCER_CC1NE);
    break;

  case 4: // Step 5: W+ U- (V Float)
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = duty;
    ccer_mask |=
        (TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC1E | TIM_CCER_CC1NE);
    break;

  case 5: // Step 6: W+ V- (U Float)
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0; // V: Low Side ON
    TIM1->CCR3 = duty;
    ccer_mask |=
        (TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC2E | TIM_CCER_CC2NE);
    break;
  }

  // 直接操作寄存器以实现快速换相
  TIM1->CCER = ccer_mask;

  HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM);
}

/**
 * @brief 关闭所有输出 (让电机线圈悬空)
 */
void Motor_Stop(void) {
  // 只保留 CC4E (ADC触发)，关闭其他所有通道的输出
  TIM1->CCER = TIM_CCER_CC4E;
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM);
}

/**
 * @brief 微秒级延时 (简单的空循环，根据主频72MHz估算)
 */
void Delay_us(uint32_t us) {
  // 72MHz下，大约 72个周期为1us。
  // 循环一次大约消耗几个周期，这里粗略估算，系数需要根据实际情况微调
  // 假设系数为 10
  for (volatile int i = 0; i < us * 10; i++) {
    __NOP();
  }
}

/**
 * @brief 让电机发出蜂鸣声
 * @param frequency: 声音频率 (Hz)，例如 2000
 * @param duration_ms: 持续时间 (ms)，例如 500
 * @param volume: 音量 (PWM占空比)，建议 50~200，太大会导致电机发热或转动
 */
void Motor_Beep(uint16_t frequency, uint16_t duration_ms, uint16_t volume) {
  if (frequency == 0)
    return;

  uint32_t period_us = 1000000 / frequency;
  // 计算周期 (微秒)
  uint32_t half_period = period_us / 2;
  // 半周期
  uint32_t cycles = (duration_ms * 1000) / period_us; // 总震动次数

  for (uint32_t i = 0; i < cycles; i++) {
    // 1. 通电 (使用 Step 0: U+ V-)
    // 注意：volume 必须很小，否则电机就转起来了！
    SixStep_Commutate(5, volume);
    Delay_us(half_period);

    // 2. 断电 (悬空)
    Motor_Stop();
    Delay_us(half_period);
  }
}

/**
 * @brief 播放乐谱
 * @param song: 乐谱数组
 * @param length: 音符数量
 * @param volume: 音量 (PWM占空比)
 */
void Play_Song(const Tone *song, uint16_t length, uint16_t volume) {
  for (uint16_t i = 0; i < length; i++) {
    if (song[i].note == REST) {
      Motor_Stop();
      HAL_Delay(song[i].duration);

    } else {
      // 播放音符
      Motor_Beep(song[i].note, song[i].duration, volume);
    }
    // 音符之间的短暂间隔，让声音更清晰
    HAL_Delay(20);
  }
  Motor_Stop();
}

// 目标转速软斜坡(避免开环->闭环突变)
static uint32_t target_rpm_ramped = 0;
static uint32_t ramp_last_ms = 0;
static const uint32_t ramp_step_rpm = 100; // 每步提升的转速
static const uint32_t ramp_period_ms = 20; // 斜坡周期

// TIM3=500Hz 固定 dt=100Hz
#define PID_DT_SEC (1.0f / 100.0f)

/**
 * @brief 速度环 PID 计算
 * @param pid PID 控制器结构体指针
 * @param target 目标转速 (RPM)
 * @param actual 实际转速 (RPM)
 * @return uint32_t 计算得到的 PWM 占空比
 * @author 18Fly (iphoneios@88.com)
 * @date 2026-02-07
 */
static uint32_t SpeedPID_Update(PID_t *pid, uint32_t target, uint32_t actual) {
  float dt = PID_DT_SEC;
  float error = (float)target - (float)actual;
  // 积分
  pid->integral += error * dt;
  // 积分限幅
  if (pid->integral > 30000.0f)
    pid->integral = 30000.0f;
  if (pid->integral < -30000.0f)
    pid->integral = -30000.0f;
  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;
  float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  float duty = out;
  if (duty < pwm_min)
    duty = pwm_min;
  if (duty > pwm_max)
    duty = pwm_max;
  return (uint32_t)duty;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU
       Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the
  Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  HAL_Delay(20000); // 上电等待 20 秒，方便调试时连接串口
  // 执行自校准，确保 ADC 有最佳的采样精度
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    // 如果这里报错，说明芯片的时钟或者 ADC 硬件有问题
    Error_Handler();
  }
  // 暴力清除所有可能遗留的标志位，以防万一
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
  // ADC 注入组中断
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_ENABLE(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);

  // static char msg[256] = {0}; 调试的时候用

  // CDC_Transmit_FS((uint8_t *)"[MAIN] 播放自检音...\n", 27);

  // // 哆 (Do) - 频率约 1046Hz
  // Motor_Beep(1046, 200, 200);
  // HAL_Delay(50);
  // // 咪 (Mi) - 频率约 1318Hz
  // Motor_Beep(1318, 200, 200);
  // HAL_Delay(50);
  // // 索 (Sol) - 频率约 1568Hz
  // Motor_Beep(1568, 200, 200);
  // HAL_Delay(50);
  // // 高音哆 (High Do) - 频率约 2093Hz
  // Motor_Beep(2093, 400, 200);

  // 播放马里奥 (节奏快)
  // Play_Song(Mario_Theme, sizeof(Mario_Theme) / sizeof(Tone), 300);

  // 播放帝国进行曲 (节奏慢)
  // Play_Song(Imperial_March, sizeof(Imperial_March) / sizeof(Tone), 300);

  // HAL_Delay(3000); // 响完停顿3s

  // 强行把转子吸到 Step 5 的位置，防止起步乱跳
  SixStep_Commutate(5, 1000);
  CDC_Transmit_FS((uint8_t *)"[MAIN] 电机已归位\n", 24);
  HAL_Delay(50); // 等待 50ms 让转子归位
  step = 0;
  pwmDuty = 500; // 初始 PWM 占空比
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  CDC_Transmit_FS((uint8_t *)"[MAIN] 开环启动中...\n", 27);
  while (1) {
    if (run_mode == 0) {
      SixStep_Commutate(step, pwmDuty);
      step++;
      if (step >= 6)
        step = 0;
      uint32_t start_tick = DWT->CYCCNT;
      uint32_t wait_cycles = step_delay * 72;
      while ((DWT->CYCCNT - start_tick) < wait_cycles) {
        __NOP();
      }
      static uint8_t speed_prescaler = 0;
      if (++speed_prescaler >= 12) { // 每转一圈(6步)由程序调整一次速度
        speed_prescaler = 0;
        uint32_t decrement = 0;
        if (step_delay > 2000)
          decrement = 40; // 低速阶段：快速加速
        else if (step_delay > 1200)
          decrement = 20; // 中速阶段
        else
          decrement = 5;      // 切换前夕：微调，极度稳定
        if (step_delay > 300) // 最小值保护，防止延时溢出
          step_delay -= decrement;
        // 补电压：速度越快 BEMF 越高，需要更大的占空比来维持电流
        if (pwmDuty < 1500)
          pwmDuty += 10;
      }

      // 切入闭环中断模式
      if (step_delay <= closed_loop_threshold) {
        last_commutation_time = DWT->CYCCNT;
        period_time = step_delay * 72;
        run_mode = 1;
        // 闭环切入过渡
        pid_enabled = 0;
        speed_pid.integral = 0.0f;
        speed_pid.prev_error = 0.0f;
        speed_pid.last_ms = HAL_GetTick();
        if (speed_pid.ki > 0.0f) {
          speed_pid.integral = (float)pwmDuty / speed_pid.ki;
        } else {
          speed_pid.integral = 0.0f;
        }
        ramp_last_ms = HAL_GetTick();
        target_rpm_ramped = 3000;
        rpm = 1500; // 估算一个初始值，避免突变
      }
    } else {
      static uint32_t last_print_time = 0;
      static uint32_t closed_loop_enter_ms = 0;
      if (closed_loop_enter_ms == 0) {
        closed_loop_enter_ms = HAL_GetTick();
      }
      // 过渡延时后启用 PID
      if (!pid_enabled &&
          (HAL_GetTick() - closed_loop_enter_ms) > pid_enable_delay_ms) {
        pid_enabled = 1;
      }
      if (HAL_GetTick() - last_print_time > 200) {
        last_print_time = HAL_GetTick();
        char rpm_msg[64];
        sprintf(rpm_msg, "%lu,%lu,%lu\n", rpm, target_rpm_ramped, pwmDuty);
        CDC_Transmit_FS((uint8_t *)rpm_msg, strlen(rpm_msg));
      }
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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

// RPM 一阶低通系数
#define RPM_LPF_ALPHA 0.05f // 稍微降低一点，让曲线更平滑
static float rpm_lpf = 0.0f;
static uint8_t pid_prescaler = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    if (run_mode == 1) {
      float rpm_raw = 0.0f;
      if (period_time > 0) {
        rpm_raw =
            (72000000.0f * 10.0f) / ((float)period_time * MOTOR_POLE_PAIRS);
      }
      if (rpm_lpf == 0.0f)
        rpm_lpf = rpm_raw;
      else
        rpm_lpf += RPM_LPF_ALPHA * (rpm_raw - rpm_lpf);
      rpm = (uint32_t)rpm_lpf;
      if (pid_enabled) {
        pid_prescaler++;
        if (pid_prescaler >= 5) // 100Hz
          pid_prescaler = 0;
        if (HAL_GetTick() - ramp_last_ms >= ramp_period_ms) {
          ramp_last_ms = HAL_GetTick();
          if (target_rpm_ramped < target_rpm) {
            target_rpm_ramped += ramp_step_rpm;
            if (target_rpm_ramped > target_rpm)
              target_rpm_ramped = target_rpm;
          } else if (target_rpm_ramped > target_rpm) {
            target_rpm_ramped -= ramp_step_rpm;
            if (target_rpm_ramped < target_rpm)
              target_rpm_ramped = target_rpm;
          }
        }
        pwmDuty =
            SpeedPID_Update(&speed_pid, (float)target_rpm_ramped, (float)rpm);
      } else {
        if (step_delay > 0) {
          float open_loop_rpm =
              60000000.0f / ((float)step_delay * MOTOR_POLE_PAIRS);
          rpm_lpf = open_loop_rpm;
        }
      }
    }
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    // 前两次采样结果依次后移
    theFirstAdcU = theSecondAdcU;
    theFirstAdcV = theSecondAdcV;
    theFirstAdcW = theSecondAdcW;

    theSecondAdcU = currentAdcU;
    theSecondAdcV = currentAdcV;
    theSecondAdcW = currentAdcW;

    currentAdcU = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    currentAdcV = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    currentAdcW = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    adcU = (theFirstAdcU + theSecondAdcU + currentAdcU) / 3;
    adcV = (theFirstAdcV + theSecondAdcV + currentAdcV) / 3;
    adcW = (theFirstAdcW + theSecondAdcW + currentAdcW) / 3;

    if (run_mode == 1) // 只有在闭环模式下才检测
    {
      //  消磁屏蔽逻辑
      uint32_t current_time_check = DWT->CYCCNT;
      uint32_t time_since_commutation =
          current_time_check - last_commutation_time;

      // 动态计算屏蔽时间：取周期的 1/4
      // 如果 period_time 为0 (刚启动)，给一个很小的安全值 (3600 cycles ≈ 50us)
      uint32_t blanking_time = (period_time > 0) ? (period_time / 4) : 3600;

      // 只有过了屏蔽期，才进行过零检测
      if (time_since_commutation >= blanking_time) {
        // 简单的虚拟中性点计算
        uint32_t neutral_point = (adcU + adcV + adcW) / 3;
        uint8_t zc_detected = 0;

        switch (step) {
        case 0:
          if (adcW < neutral_point)
            zc_detected = 1;
          break;
        case 1:
          if (adcV > neutral_point)
            zc_detected = 1;
          break;
        case 2:
          if (adcU < neutral_point)
            zc_detected = 1;
          break;
        case 3:
          if (adcW > neutral_point)
            zc_detected = 1;
          break;
        case 4:
          if (adcV < neutral_point)
            zc_detected = 1;
          break;
        case 5:
          if (adcU > neutral_point)
            zc_detected = 1;
          break;
        default:
          step = 0;
          break;
        }

        if (zc_detected) {
          // 转速计算 (6步平均法)
          uint32_t current_time = DWT->CYCCNT;
          uint32_t delta_time = current_time - last_commutation_time;
          last_commutation_time = current_time;

          static uint32_t sum_delta_time = 0;
          static uint8_t step_counter = 0;

          // 异常值过滤：
          // 下限 3600 (50us): 过滤高频噪声
          // 上限 3600000 (50ms): 过滤掉丢步导致的超长间隔
          if (delta_time > 3600 && delta_time < 3600000) {
            sum_delta_time += delta_time;
            step_counter++;
          } else {
            // 遇到异常值，重置统计，防止脏数据污染
            sum_delta_time = 0;
            step_counter = 0;
          }

          // 每 6 步（一圈电角度）更新一次全局 period_time
          if (step_counter >= 6) {
            uint32_t avg_time = sum_delta_time / 6;
            // 低通滤波：使数值变化更平滑
            if (period_time == 0)
              period_time = avg_time;
            else
              period_time = (period_time + avg_time) >> 1;
            sum_delta_time = 0;
            step_counter = 0;
          }

          //  执行换相
          step++;
          if (step >= 6)
            step = 0;
          SixStep_Commutate(step, pwmDuty);
        }
      }
    }
    // 统一清除标志位 (必须执行)
    hadc->Instance->SR = ~(ADC_SR_JEOC);
    hadc->Instance->CR1 |= ADC_CR1_JEOCIE;
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
  file,      line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
