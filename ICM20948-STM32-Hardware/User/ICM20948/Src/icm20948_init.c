#include "icm20948_init.h"

// --- 内部辅助函数 ---

// 写寄存器：片选 -> 地址 -> 数据 -> 释放
static void icm20948_write_reg(uint8_t reg, uint8_t value) {
  ICM20948_Select();
  SPI1_ReadWriteByte(reg);
  SPI1_ReadWriteByte(value);
  ICM20948_Deselect();
  // 某些操作后建议稍作延时，防止指令发送太快传感器处理不过来
  // HAL_Delay(1) 在高速连续配置时可能有帮助，视情况而定
}

// 读寄存器：片选 -> 地址(读位) -> 读取数据 -> 释放
static uint8_t icm20948_read_reg(uint8_t reg) {
  uint8_t value;
  ICM20948_Select();
  SPI1_ReadWriteByte(reg | 0x80); // 设置读标志位
  value = SPI1_ReadWriteByte(0x00);
  ICM20948_Deselect();
  return value;
}

// 切换 Bank：封装常用操作
static void icm20948_select_bank(uint8_t bank) {
  icm20948_write_reg(UB_RegBankSel_RW, bank << 4);
}

static void icm20948_mag_write_register(uint8_t reg, uint8_t data) {
  icm20948_select_bank(3);

  // 配置 Slave 4 地址 (I2C写: 地址仅包含ID)
  icm20948_write_reg(UB3_I2cSlv4Addr_RW, AK09916_I2C_ADDR);
  // 配置目标寄存器
  icm20948_write_reg(UB3_I2cSlv4Reg_RW, reg);
  // 配置数据
  icm20948_write_reg(UB3_I2cSlv4Do_RW, data);
  // 启动传输 (EN | IntEn | Dly:0)
  icm20948_write_reg(UB3_I2cSlv4Ctrl_RW, 0x80 | 0x00);

  // 等待传输完成 (简单延时处理，实际可以用轮询 I2C_MST_STATUS)
  HAL_Delay(10);

  icm20948_select_bank(0);
}

// --- 主要功能函数 ---

void ICM20948_Select() {
  HAL_GPIO_WritePin(ICM20948_CS_PORT, ICM20948_CS_PIN, GPIO_PIN_RESET);
}

void ICM20948_Deselect() {
  HAL_GPIO_WritePin(ICM20948_CS_PORT, ICM20948_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPI1_ReadWriteByte(uint8_t txData) {
  uint8_t rxData = 0;
  if (HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, HAL_MAX_DELAY) !=
      HAL_OK) {
    Error_Handler();
  }
  return rxData;
}

uint8_t ICM20948_WAI(void) {
  icm20948_select_bank(0);
  return icm20948_read_reg(UB0_WhoAmI_R);
}

int icm20948_initialize(void) {
  uint8_t id = ICM20948_WAI();
  if (id != ICM20948_TRUE_ID) {
    return -1;
  }

  // 复位设备
  icm20948_write_reg(UB0_PwrMgmt1_RW, 0x80);
  HAL_Delay(100);
  icm20948_write_reg(UB0_PwrMgmt1_RW, 0x01); // Auto Clock
  HAL_Delay(10);

  // 核心修改：启用 SPI 同时也启用 I2C Master
  // Bit 4: I2C_IF_DIS (禁用外部I2C接口，这是必须的，因为我们用SPI)
  // Bit 5: I2C_MST_EN (启用内部 I2C Master，用于连接磁力计)
  icm20948_write_reg(UB0_UserCtrl_RW, 0x20 | 0x10);
  HAL_Delay(10);

  // 配置 I2C Master 时钟
  icm20948_select_bank(3);
  icm20948_write_reg(UB3_I2cMstCtrl_RW, I2C_MST_CLK_345_KHZ);
  HAL_Delay(10);
  icm20948_select_bank(0);

  // 初始化磁力计 (AK09916)
  // 软复位磁力计
  icm20948_mag_write_register(AK09916_CNTL3, 0x01);
  HAL_Delay(100);

  // 设置为 100Hz 连续测量模式 (Mode 4)
  icm20948_mag_write_register(AK09916_CNTL2, 0x08);
  HAL_Delay(10);

  // 配置 I2C Master 自动读取磁力计数据 (Slave 0)
  icm20948_select_bank(3);

  // Slave 0 地址: AK09916 | Read Flag (0x80)
  icm20948_write_reg(UB3_I2cSlv0Addr_RW, AK09916_I2C_ADDR | 0x80);

  // 起始寄存器: AK09916_ST1 (0x10) - 状态寄存器1
  // AK09916 要求必须读完 ST1 -> Data(6) -> ST2，才能更新下一次数据
  icm20948_write_reg(UB3_I2cSlv0Reg_RW, AK09916_ST1);

  // 控制寄存器: Enable | Length=9 (ST1 + 6 Data + ST2)
  // Bit 7: EN, Bit 3-0: Length
  icm20948_write_reg(UB3_I2cSlv0Ctrl_RW, 0x80 | 0x09);

  icm20948_select_bank(0);

  // --- 陀螺仪和加速度计配置 (保持原样) ---
  icm20948_select_bank(2);
  // 陀螺仪 ±2000dps
  icm20948_write_reg(UB2_GyroConfig1_RW, 0b00100111);
  // 加速度计 ±16g
  icm20948_write_reg(UB2_AccelConfig_RW, 0b00100111);
  // ODR = 1100 / (1 + DIV) -> 1100/1 = 1100Hz -> DIV=0
  icm20948_write_reg(UB2_GyroSmplrtDiv_RW, 0);
  icm20948_write_reg(UB2_AccelSmplrtDiv2_RW, 0);
  icm20948_select_bank(0);

  return 0;
}

void icm20948_fifo_enable(void) {
  icm20948_select_bank(0);

  // 关闭 FIFO, 保持 I2C Master 和 SPI Interface 逻辑
  // User Ctrl: FIFO_EN=0, I2C_MST_EN=1 (Bit5), I2C_IF_DIS=1 (Bit4) -> 0x30
  icm20948_write_reg(UB0_UserCtrl_RW, 0x30);
  HAL_Delay(10);

  // 复位 FIFO
  // User Ctrl: FIFO_RST=1 (Bit2) | 0x30
  icm20948_write_reg(UB0_UserCtrl_RW, 0x30 | 0x04);
  HAL_Delay(10);

  // 配置 FIFO 模式 (Stream)
  icm20948_write_reg(UB0_FifoMode_RW, 0x00);

  // FIFO 数据源选择
  // En2: Accel + Gyro + Temp(可选) -> 0x1E (Accel + Gyro)
  icm20948_write_reg(UB0_FifoEn2_RW, 0x1E);

  // En1: Slave 0 (磁力计数据) -> Bit 0
  icm20948_write_reg(UB0_FifoEn1_RW, 0x01);
  HAL_Delay(1);

  // 开启 FIFO
  // User Ctrl: FIFO_EN=1 (Bit6) | 0x30
  icm20948_write_reg(UB0_UserCtrl_RW, 0x40 | 0x30);
  HAL_Delay(10);
}

void icm20948_read_fifo(uint8_t *buffer) {
  icm20948_select_bank(0);

  uint8_t fifoCountH = icm20948_read_reg(UB0_FifoCountH_R);
  uint8_t fifoCountL = icm20948_read_reg(UB0_FifoCountL_R);
  uint16_t fifoCount = (fifoCountH << 8) | fifoCountL;

  if (fifoCount >= 21) {
    ICM20948_Select();
    SPI1_ReadWriteByte(UB0_FifoRW_RW | 0x80);

    // 读取 21 字节
    for (uint8_t i = 0; i < 21; i++) {
      buffer[i] = SPI1_ReadWriteByte(0x00);
    }
    ICM20948_Deselect();
  }
}

void icm20948_read_raw_data(uint8_t *buffer) {
  icm20948_select_bank(0);

  ICM20948_Select();
  // 从 Accel X High (0x2D) 开始连读
  // 0x2D 到 0x38 (Accel 6 + Gyro 6 = 12 bytes)
  // 0x3B (EXT_SLV_SENS_DATA_00) 开始是磁力计数据
  // 注意中间有 Temp (0x39, 0x3A)
  // 一次读完: 12 (AG) + 2 (Temp) + 9 (Mag) = 23 bytes
  SPI1_ReadWriteByte(0x2D | 0x80);

  for (uint8_t i = 0; i < 23; i++) {
    buffer[i] = SPI1_ReadWriteByte(0x00);
  }
  ICM20948_Deselect();

  // buffer 结构:
  // 0-5: Accel
  // 6-11: Gyro
  // 12-13: Temp
  // 14: Mag ST1 (Status 1)
  // 15-20: Mag Data (Little Endian!)
  // 21: Mag ST2
}

uint8_t ICM20948_IsDataReady(void) {
  uint8_t status = 0;
  icm20948_select_bank(0);

  ICM20948_Select();
  SPI1_ReadWriteByte(UB0_IntStatus1_R | 0x80); // 读地址
  status = SPI1_ReadWriteByte(0x00);
  ICM20948_Deselect();

  // Bit 0 是 RAW_DATA_RDY_INT
  return (status & 0x01);
}
