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
#include "gpio.h"
#include "spi.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DeviceCalibration.h"
#include "MahonyAHRS.h"
#include "icm20948_init.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef char *restrict cstr_ptr;
typedef const char *restrict ccstr_ptr;

typedef struct {
  float offset[3];
  float scale[3];
  uint32_t magic_num; // 用于判断 Flash 是否为空
} MagSaveData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 强转
#define INT_PART(x) ((int)(x))
// 取绝对值 -> 减去整数部分 -> 乘100 -> 强转
#define FRAC_PART_100(x) ((int)((fabs(x) - (int)fabs(x)) * 100))

#define FLASH_SAVE_ADDR 0x0800F800 // STM32F103C8T6 最后一页 2KB 用于保存数据

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Mahony_Handle_t mahonyFilter;
MagCalib_t magCalib;

float magOffset[3];
float magScale[3];

int16_t gyro_bias_x;
int16_t gyro_bias_y;
int16_t gyro_bias_z;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief 保存校准数据到 Flash
 * @param offset 偏置数组
 * @param scale 比例数组
 * @author 18Fly (iphoneios@88.com)
 * @date 2026-02-03
 */
void Save_Calib_To_Flash(float offset[3], float scale[3]) {
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;

  // 擦除一页
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_SAVE_ADDR;
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return;
  }

  // 写入数据 (每次写入 32位 / 1个Word)
  MagSaveData_t data;
  for (int i = 0; i < 3; i++) {
    data.offset[i] = offset[i];
    data.scale[i] = scale[i];
  }
  data.magic_num = 0xDEADBEEF; // 标记位

  uint32_t *pData = (uint32_t *)&data;
  int numWords = sizeof(MagSaveData_t) / 4;

  for (int i = 0; i < numWords; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_SAVE_ADDR + i * 4,
                          pData[i]) != HAL_OK) {
      break;
    }
  }
  HAL_FLASH_Lock(); // 上锁
}

/**
 * @brief 从 Flash 加载校准数据
 * @param offset 偏置数组
 * @param scale 比例数组
 * @author 18Fly (iphoneios@88.com)
 * @date 2026-02-03
 */
void Load_Calib_From_Flash(float offset[3], float scale[3]) {
  MagSaveData_t *pData = (MagSaveData_t *)FLASH_SAVE_ADDR;

  if (pData->magic_num == 0xDEADBEEF) {
    for (int i = 0; i < 3; i++) {
      offset[i] = pData->offset[i];
      scale[i] = pData->scale[i];
    }
    magCalib.is_calibrated = 1;
  } else {
    for (int i = 0; i < 3; i++) {
      offset[i] = 0.0f;
      scale[i] = 1.0f;
    }
  }
}

/**
 * @brief 上电执行陀螺仪静态校准
 * @author 18Fly (iphoneios@88.com)
 * @date 2026-02-03
 */
void Perform_Gyro_Calibration(void) {
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  uint8_t valueBuffer[24] = {0};

  const int sample_count = 1000;

  for (int i = 0; i < sample_count; i++) {
    while (!ICM20948_IsDataReady()) {
    };
    icm20948_read_raw_data((uint8_t *)valueBuffer);
    int16_t raw_gx = (int16_t)((valueBuffer[6] << 8) | valueBuffer[7]);
    int16_t raw_gy = (int16_t)((valueBuffer[8] << 8) | valueBuffer[9]);
    int16_t raw_gz = (int16_t)((valueBuffer[10] << 8) | valueBuffer[11]);

    // 累加陀螺仪数据
    sum_gx += raw_gx;
    sum_gy += raw_gy;
    sum_gz += raw_gz;
  }

  // 得到零偏
  gyro_bias_x = (int16_t)(sum_gx / sample_count);
  gyro_bias_y = (int16_t)(sum_gy / sample_count);
  gyro_bias_z = (int16_t)(sum_gz / sample_count);
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
   * Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the
   * Systick.
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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(5000);

  // uint8_t strBuffer[128] = {0};
  uint8_t valueBuffer[24] = {0};
  uint8_t data_buffer[16];
  uint8_t strBuffer[128] = {0};
  // uint8_t print_counter = 0;

  icm20948_initialize();
  // icm20948_fifo_enable();

  Mag_Calib_Init(&magCalib);
  Mahony_Init(&mahonyFilter, DEFAULT_SAMPLE_FREQ);

  Load_Calib_From_Flash(magOffset, magScale);
  CDC_Transmit_FS((uint8_t *)"硬/软铁校准数据加载中...\n", 36);
  sprintf((char *)strBuffer,
          "校准数据加载完毕！\n"
          "偏置: [%d.%02d, %d.%02d, %d.%02d]\n"
          "比例: [%d.%02d, %d.%02d, %d.%02d]\n",

          // 偏置 X
          INT_PART(magOffset[0]), FRAC_PART_100(magOffset[0]),
          // 偏置 Y
          INT_PART(magOffset[1]), FRAC_PART_100(magOffset[1]),
          // 偏置 Z
          INT_PART(magOffset[2]), FRAC_PART_100(magOffset[2]),

          // 比例 X
          INT_PART(magScale[0]), FRAC_PART_100(magScale[0]),
          // 比例 Y
          INT_PART(magScale[1]), FRAC_PART_100(magScale[1]),
          // 比例 Z
          INT_PART(magScale[2]), FRAC_PART_100(magScale[2]));

  CDC_Transmit_FS(strBuffer, strlen((char *)strBuffer));
  HAL_Delay(100);
  CDC_Transmit_FS((uint8_t *)"开始陀螺仪上电校准，请保持静止...\n", 50);
  Perform_Gyro_Calibration();
  CDC_Transmit_FS((uint8_t *)"陀螺仪校准完成！\n", 26);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_tick = HAL_GetTick();
  while (1) {
    if (!ICM20948_IsDataReady()) {
      continue;
    }
    icm20948_read_raw_data(valueBuffer);
    int16_t raw_ax = (int16_t)((valueBuffer[0] << 8) | valueBuffer[1]);
    int16_t raw_ay = (int16_t)((valueBuffer[2] << 8) | valueBuffer[3]);
    int16_t raw_az = (int16_t)((valueBuffer[4] << 8) | valueBuffer[5]);
    int16_t raw_gx = (int16_t)((valueBuffer[6] << 8) | valueBuffer[7]);
    int16_t raw_gy = (int16_t)((valueBuffer[8] << 8) | valueBuffer[9]);
    int16_t raw_gz = (int16_t)((valueBuffer[10] << 8) | valueBuffer[11]);
    int16_t raw_mx = (int16_t)((valueBuffer[16] << 8) | valueBuffer[15]);
    int16_t raw_my = (int16_t)((valueBuffer[18] << 8) | valueBuffer[17]);
    int16_t raw_mz = (int16_t)((valueBuffer[20] << 8) | valueBuffer[19]);

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET &&
        !magCalib.is_calibrated) {

      Mag_Calib_Update(&magCalib, (float)raw_mx, (float)raw_my, (float)raw_mz);
      sprintf((char *)strBuffer, "校准中...样本数: %d\n",
              magCalib.sample_count);
      CDC_Transmit_FS(strBuffer, strlen((char *)strBuffer));
      HAL_Delay(1);

    } else if (!magCalib.is_calibrated) {

      Mag_Calib_Calc(&magCalib);
      magOffset[0] = magCalib.offset[0];
      magOffset[1] = magCalib.offset[1];
      magOffset[2] = magCalib.offset[2];
      magScale[0] = magCalib.scale[0];
      magScale[1] = magCalib.scale[1];
      magScale[2] = magCalib.scale[2];
      Save_Calib_To_Flash(magOffset, magScale);
      CDC_Transmit_FS((uint8_t *)"保存校准数据到内部Flash！\n", 37);
      HAL_Delay(500);

    } else {

      uint32_t current_tick = HAL_GetTick();
      if (current_tick == last_tick) {
        // 时间还没过 1ms，不要运行积分算法，否则会引入巨大误差
        HAL_Delay(0);
        continue;
      }

      float dt = (current_tick - last_tick) / 1000.0f;
      last_tick = current_tick;
      mahonyFilter.invSampleFreq = dt;

      // 去除陀螺仪零偏
      raw_gx -= gyro_bias_x;
      raw_gy -= gyro_bias_y;
      raw_gz -= gyro_bias_z;

      // 陀螺仪范围 ±2000dps，灵敏度系数为 16.4 LSB/dps
      // Mahony 算法需要弧度制 (rad/s)
      float gx_rad = ((float)raw_gx / 16.4f) * (3.14159f / 180.0f);
      float gy_rad = ((float)raw_gy / 16.4f) * (3.14159f / 180.0f);
      float gz_rad = ((float)raw_gz / 16.4f) * (3.14159f / 180.0f);

      // 磁力计硬铁 + 软铁校准
      float mx_calib = (raw_mx - magOffset[0]) * magScale[0];
      float my_calib = (raw_my - magOffset[1]) * magScale[1];
      float mz_calib = (raw_mz - magOffset[2]) * magScale[2];

      float align_mx = mx_calib;
      float align_my = -my_calib;
      float align_mz = -mz_calib;

      Mahony_Update9DOF(&mahonyFilter, gx_rad, gy_rad, gz_rad, raw_ax, raw_ay,
                        raw_az, align_mx, align_my, align_mz);

      Mahony_ComputeEuler(&mahonyFilter);

      // 如果 raw_mx全是0，说明磁力计压根没读到，这是漂移的根本原因
      // 取消注释检查磁力计数据
      // sprintf((char *)strBuffer, "Mag Raw: %d, %d, %d\n", raw_mx, raw_my,
      //         raw_mz);
      // CDC_Transmit_FS(strBuffer, strlen((char *)strBuffer));

      *(float *)(data_buffer + 0) = mahonyFilter.roll;
      *(float *)(data_buffer + 4) = mahonyFilter.pitch;
      *(float *)(data_buffer + 8) = mahonyFilter.yaw;
      data_buffer[12] = 0x00;
      data_buffer[13] = 0x00;
      data_buffer[14] = 0x80;
      data_buffer[15] = 0x7f;
      CDC_Transmit_FS(data_buffer, 16);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

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
    CDC_Transmit_FS(
        (uint8_t *)"进入错误处理！\n",
        23); // 这里的话建议弄个枚举类型，从'A'的ASCII值开始，
             // 形参传过来作为字符直接打印，不需要再转字符串了
             // 我懒得弄了(*^_^*)😴，如果用不了硬件调试的可以这样弄一下
    HAL_Delay(1000);
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
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
