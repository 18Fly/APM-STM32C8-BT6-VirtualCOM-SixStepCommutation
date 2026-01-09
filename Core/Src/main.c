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
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
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

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

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
uint32_t step_delay = 3000; // 初始换相延时 (us) - 越小转越快

// 闭环控制参数
volatile uint8_t run_mode = 0;               // 0: 开环启动, 1: 闭环运行
volatile uint32_t bemf_threshold = 0;        // BEMF 过零阈值 (Vbus/2)
volatile uint32_t commutation_timer = 0;     // 用于计算换相间隔
volatile uint32_t zc_timestamp = 0;          // 过零点时刻
volatile uint32_t last_commutation_time = 0; // 上一次换相时刻
volatile uint32_t period_time = 0;           // 换相周期 (60度电角度时间)
uint32_t closed_loop_threshold = 1000;       // 闭环切换阈值
// volatile uint32_t isrCounter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
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
  case 0:              // Step 1: U+ V- (W Float)
    TIM1->CCR1 = duty; // U: PWM
    TIM1->CCR2 = 0;    // V: Low Side ON (CCR=0 -> CH2=0, CH2N=1)
    TIM1->CCR3 = 0;    // W: Float (CCR无所谓，因为输出被禁用了)
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

  case 3:           // Step 4: V+ U- (W Float)
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

  uint32_t period_us = 1000000 / frequency;           // 计算周期 (微秒)
  uint32_t half_period = period_us / 2;               // 半周期
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

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
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

  // uint32_t test_pulse = 1800;                               // 3600 / 2 =
  // 1800
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, test_pulse); // U相
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, test_pulse); // V相
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, test_pulse); // W相

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  __HAL_TIM_MOE_ENABLE(&htim1);

  // static char msg[256] = {0}; 调试的时候用

  CDC_Transmit_FS((uint8_t *)"[MAIN] 播放自检音...\n", 27);

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
  Play_Song(Mario_Theme, sizeof(Mario_Theme) / sizeof(Tone), 300);

  // 播放帝国进行曲 (节奏慢)
  // Play_Song(Imperial_March, sizeof(Imperial_March) / sizeof(Tone), 300);

  HAL_Delay(3000); // 响完停顿3s

  // 强行把转子吸到 Step 5 的位置，防止起步乱跳
  SixStep_Commutate(5, 1000);
  CDC_Transmit_FS((uint8_t *)"[MAIN] 电机已归位\n", 24);
  HAL_Delay(50); // 等待 50ms 让转子归位
  step = 0;      // 归位后从 0 开始
  pwmDuty = 700; // 初始 PWM 占空比
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  CDC_Transmit_FS((uint8_t *)"[MAIN] 开环启动中...\n", 27);
  while (1) {
    // // 执行换相
    // SixStep_Commutate(step, pwmDuty);

    // // 步进
    // step++;
    // if (step >= 6)
    //   step = 0;

    // // 简单的开环加速逻辑
    // static uint8_t speed_prescaler = 0;
    // if (++speed_prescaler >= 2) // 每换相 6 次（转一圈）才加速一次
    // {
    //   speed_prescaler = 0;

    //   // 只有没达到最高速时才加速
    //   if (step_delay > min_delay)
    //   {
    //     step_delay -= 20; // 缓慢减小延时

    //     // 速度越快(delay越小)，需要的电压(pwmDuty)越高
    //     // 简单的线性补偿：防止高速时力矩不足
    //     if (pwmDuty < 3500)
    //     {
    //       pwmDuty += 15;
    //     }
    //   }
    // }

    // // 延时 (模拟换相间隔)
    // // 注意：HAL_Delay 是毫秒级，太慢了。这里用简单的空循环模拟微秒延时
    // // 实际项目中应该用定时器中断来控制换相时间
    // for (volatile int i = 0; i < step_delay * 5; i++)
    // {
    //   __NOP();
    // }

    if (run_mode == 0) {
      SixStep_Commutate(step, pwmDuty);
      step++;
      if (step >= 6)
        step = 0;

      // 简单的开环加速
      static uint8_t speed_prescaler = 0;
      if (++speed_prescaler >= 3) {
        speed_prescaler = 0;
        if (step_delay > 200)
          step_delay -= 10; // 加速
        if (pwmDuty < 3000)
          pwmDuty += 5;
        // 补电压
        // 这里我调了几次可以设置大一点，防止过渡到闭环时转矩不够，影响反向电动势过零点检测
      }
      // 延时，尽量别放USB_CDC或者其他严重影响时序的代码在这里
      for (volatile int i = 0; i < step_delay * 5; i++)
        __NOP();

      // 切入闭环中断模式
      if (step_delay <= closed_loop_threshold) {
        last_commutation_time = DWT->CYCCNT;
        period_time = step_delay * 35;
        run_mode = 1; // 开启中断里的换相逻辑
      }
    }
    //  闭环运行 (高速)
    else {
      // 简单的油门控制：一直加速到满油门
      // if (pwmDuty < 3000) {
      //   pwmDuty += 2;
      //   HAL_Delay(3);
      // } else {
      //   static uint32_t last_print_time = 0;

      //   // 超时检测 (堵转保护)
      //   uint32_t now_dwt = DWT->CYCCNT;
      //   // 72MHz下，36M计数约等于0.5秒
      //   if ((now_dwt - last_commutation_time) > 36000000) {
      //     period_time = 0; // 超过0.5秒没换相，强制归零
      //   }

      //   // 定时打印 (每200ms)
      //   if (HAL_GetTick() - last_print_time > 200) {
      //     last_print_time = HAL_GetTick();

      //     if (period_time > 0) {
      //       // RPM = (72,000,000 * 60) / (period_time * 6 * POLE_PAIRS)
      //       // 简化公式: RPM = 720,000,000 / (period_time * POLE_PAIRS)
      //       uint32_t rpm = 720000000 / (period_time * MOTOR_POLE_PAIRS);

      //       char rpm_msg[64];
      //       sprintf(rpm_msg, "[RPM] Speed: %lu | Duty: %lu\n", rpm, pwmDuty);
      //       CDC_Transmit_FS((uint8_t *)rpm_msg, strlen(rpm_msg));
      //     } else {
      //       CDC_Transmit_FS((uint8_t *)"[RPM] Speed: 0 (STOP)\n", 22);
      //     }
      //   }
      static uint32_t last_print_time = 0;

      // 超时检测 (堵转保护)
      uint32_t now_dwt = DWT->CYCCNT;
      // 72MHz下，36M计数约等于0.5秒
      if ((now_dwt - last_commutation_time) > 36000000) {
        period_time = 0; // 超过0.5秒没换相，强制归零
      }

      // 定时打印 (每200ms)
      if (HAL_GetTick() - last_print_time > 200) {
        last_print_time = HAL_GetTick();

        if (period_time > 0) {
          // RPM = (72,000,000 * 60) / (period_time * 6 * POLE_PAIRS)
          // 简化公式: RPM = 720,000,000 / (period_time * POLE_PAIRS)
          uint32_t rpm = 720000000 / (period_time * MOTOR_POLE_PAIRS);

          char rpm_msg[64];
          sprintf(rpm_msg, "[RPM] Speed: %lu | Duty: %lu\n", rpm, pwmDuty);
          CDC_Transmit_FS((uint8_t *)rpm_msg, strlen(rpm_msg));
        } else {
          CDC_Transmit_FS((uint8_t *)"[RPM] Speed: 0 (STOP)\n", 22);
        }
      }
      // else
      // {
      // static uint32_t last_print_time = 0;

      // //  新增：超时检测
      // // 获取当前 DWT 时间
      // uint32_t now_dwt = DWT->CYCCNT;
      // // 计算距离上次换相过去了多久
      // uint32_t time_since_last_commutation = now_dwt - last_commutation_time;

      // // 72MHz 主频下，36,000,000 约为 0.5 秒
      // // 如果超过 0.5 秒没有换相，认为电机已停止
      // if (time_since_last_commutation > 36000000)
      // {
      //   period_time = 0; // 强制归零
      // }
      // // ==

      // if (HAL_GetTick() - last_print_time > 200)
      // {
      //   last_print_time = HAL_GetTick();

      //   // 防止除以0
      //   if (period_time > 0)
      //   {
      //     // 72MHz主频: 72,000,000
      //     // 公式: RPM = (72000000 * 60) / (period_time * 6 * POLE_PAIRS)
      //     // 简化: RPM = 720000000 / (period_time * POLE_PAIRS)
      //     uint32_t rpm = 720000000 / (period_time * MOTOR_POLE_PAIRS);

      //     char rpm_msg[64];
      //     sprintf(rpm_msg, "[MAIN/Motor] 转速: %lu r/min | 占空比: %lu\n",
      //     rpm, pwmDuty); CDC_Transmit_FS((uint8_t *)rpm_msg, strlen((const
      //     char *)rpm_msg));
      //   }
      //   else
      //   {
      //     // 如果 period_time 为 0，打印 0 转速
      //     char rpm_msg[64];
      //     sprintf(rpm_msg, "[MAIN/Motor] 转速: 0 r/min | 占空比: %lu
      //     (STOP)\n", pwmDuty); CDC_Transmit_FS((uint8_t *)rpm_msg,
      //     strlen((const char *)rpm_msg));
      //   }
      // }
      // }
      // 如果电机卡死或停转（长时间没进中断），需要有保护逻辑切回开环
      // 这里暂时省略，先跑通再说
    }

    // 打印调试信息 (降低打印频率，否则会卡死电机)
    // static uint32_t print_cnt = 0;
    // if (print_cnt++ > 100)
    // {
    //   print_cnt = 0;
    //   // 打印当前悬空相的电压，用于观察 BEMF
    //   // 根据 step 判断哪一相悬空
    //   uint32_t floating_adc = 0;
    //   char phase_char = ' ';
    //   if (step == 0 || step == 3)
    //   {
    //     floating_adc = adcW;
    //     phase_char = 'W';
    //   } // W float
    //   if (step == 1 || step == 4)
    //   {
    //     floating_adc = adcV;
    //     phase_char = 'V';
    //   } // V float
    //   if (step == 2 || step == 5)
    //   {
    //     floating_adc = adcU;
    //     phase_char = 'U';
    //   } // U float

    //   sprintf(msg, "Step:%d | Delay:%lu | Float(%c):%lu\n", step, step_delay,
    //   phase_char, floating_adc); CDC_Transmit_FS((uint8_t *)msg,
    //   strlen((const char *)msg));
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
              period_time = (period_time * 3 + avg_time) / 4;

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
  /* User can add his own implementation to report the HAL error return state */
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
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
