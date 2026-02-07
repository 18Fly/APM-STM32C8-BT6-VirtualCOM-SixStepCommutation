#ifndef __ICM20948_DEF_H
#define __ICM20948_DEF_H

/* ICM20948 Register Definitions */

/* Bank Selection */
#define UB_RegBankSel_RW 0x7F

/* User Bank 0 Registers Map */

#define UB0_WhoAmI_R 0x00
#define UB0_UserCtrl_RW 0x03
#define UB0_LpConfig_RW 0x05
#define UB0_PwrMgmt1_RW 0x06
#define UB0_PwrMgmt2_RW 0x07

/* Interrupts */
#define UB0_IntPinCfg_RW 0x0F
#define UB0_IntEnable_RW 0x10
#define UB0_IntEnable1_RW 0x11
#define UB0_IntEnable2_RW 0x12
#define UB0_IntEnable3_RW 0x13

/* I2C Master Status */
#define UB0_I2cMstStatus_R 0x17

/* Interrupt Status (R/C treated as R) */
#define UB0_IntStatus_R 0x19
#define UB0_IntStatus1_R 0x1A
#define UB0_IntStatus2_R 0x1B
#define UB0_IntStatus3_R 0x1C

/* Delay Time */
#define UB0_DelayTimeH_R 0x28
#define UB0_DelayTimeL_R 0x29

/* Accelerometer Data */
#define UB0_AccelXOutH_R 0x2D
#define UB0_AccelXOutL_R 0x2E
#define UB0_AccelYOutH_R 0x2F
#define UB0_AccelYOutL_R 0x30
#define UB0_AccelZOutH_R 0x31
#define UB0_AccelZOutL_R 0x32

/* Gyroscope Data */
#define UB0_GyroXOutH_R 0x33
#define UB0_GyroXOutL_R 0x34
#define UB0_GyroYOutH_R 0x35
#define UB0_GyroYOutL_R 0x36
#define UB0_GyroZOutH_R 0x37
#define UB0_GyroZOutL_R 0x38

/* Temperature Data */
#define UB0_TempOutH_R 0x39
#define UB0_TempOutL_R 0x3A

/* External Slave Sense Data (Partial) */
#define UB0_ExtSlvSensData00_R 0x3B
#define UB0_ExtSlvSensData01_R 0x3C
// ... (0x3D to 0x51 omitted for brevity)
#define UB0_ExtSlvSensData23_R 0x52

/* FIFO Enable & Control */
#define UB0_FifoEn1_RW 0x66
#define UB0_FifoEn2_RW 0x67
#define UB0_FifoRst_RW 0x68
#define UB0_FifoMode_RW 0x69
#define UB0_FifoCountH_R 0x70
#define UB0_FifoCountL_R 0x71
#define UB0_FifoRW_RW 0x72
#define UB0_DataRdyStatus_R 0x74
#define UB0_FifoCfg_RW 0x76

// --------------------------------------

/* User Bank 1 Register Map */

/* Self-Test Gyroscope */
#define UB1_SelfTestXGyro_RW 0x02
#define UB1_SelfTestYGyro_RW 0x03
#define UB1_SelfTestZGyro_RW 0x04

/* Self-Test Accelerometer */
#define UB1_SelfTestXAccel_RW 0x0E
#define UB1_SelfTestYAccel_RW 0x0F
#define UB1_SelfTestZAccel_RW 0x10

/* Offset Accelerometer */
#define UB1_XaOffsH_RW 0x14
#define UB1_XaOffsL_RW 0x15
#define UB1_YaOffsH_RW 0x17
#define UB1_YaOffsL_RW 0x18
#define UB1_ZaOffsH_RW 0x1A
#define UB1_ZaOffsL_RW 0x1B

/* Timebase Correction */
#define UB1_TimebaseCorrectionPll_RW 0x28

// --------------------------------------

/* User Bank 2 Register Map */

/* Gyroscope Configuration */
#define UB2_GyroSmplrtDiv_RW 0x00
#define UB2_GyroConfig1_RW 0x01
#define UB2_GyroConfig2_RW 0x02

/* Gyroscope Offset */
#define UB2_XgOffsUserH_RW 0x03
#define UB2_XgOffsUserL_RW 0x04
#define UB2_YgOffsUserH_RW 0x05
#define UB2_YgOffsUserL_RW 0x06
#define UB2_ZgOffsUserH_RW 0x07
#define UB2_ZgOffsUserL_RW 0x08

/* ODR Alignment */
#define UB2_OdrAlignEn_RW 0x09

/* Accelerometer Configuration */
#define UB2_AccelSmplrtDiv1_RW 0x10
#define UB2_AccelSmplrtDiv2_RW 0x11
#define UB2_AccelIntelCtrl_RW 0x12
#define UB2_AccelWomThr_RW 0x13
#define UB2_AccelConfig_RW 0x14
#define UB2_AccelConfig2_RW 0x15

/* FSYNC Configuration */
#define UB2_FsyncConfig_RW 0x52

/* Temperature Configuration */
#define UB2_TempConfig_RW 0x53

/* Mod Control */
#define UB2_ModCtrlUsr_RW 0x54

// --------------------------------------

/* User Bank 3 Register Map */

/* I2C Master Control */
#define UB3_I2cMstOdrConfig_RW 0x00
#define UB3_I2cMstCtrl_RW 0x01
#define UB3_I2cMstDelayCtrl_RW 0x02

/* I2C Slave 0 Control */
#define UB3_I2cSlv0Addr_RW 0x03
#define UB3_I2cSlv0Reg_RW 0x04
#define UB3_I2cSlv0Ctrl_RW 0x05
#define UB3_I2cSlv0Do_RW 0x06

/* I2C Slave 1 Control */
#define UB3_I2cSlv1Addr_RW 0x07
#define UB3_I2cSlv1Reg_RW 0x08
#define UB3_I2cSlv1Ctrl_RW 0x09
#define UB3_I2cSlv1Do_RW 0x0A

/* I2C Slave 2 Control */
#define UB3_I2cSlv2Addr_RW 0x0B
#define UB3_I2cSlv2Reg_RW 0x0C
#define UB3_I2cSlv2Ctrl_RW 0x0D
#define UB3_I2cSlv2Do_RW 0x0E

/* I2C Slave 3 Control */
#define UB3_I2cSlv3Addr_RW 0x0F
#define UB3_I2cSlv3Reg_RW 0x10
#define UB3_I2cSlv3Ctrl_RW 0x11
#define UB3_I2cSlv3Do_RW 0x12

/* I2C Slave 4 Control */
#define UB3_I2cSlv4Addr_RW 0x13
#define UB3_I2cSlv4Reg_RW 0x14
#define UB3_I2cSlv4Ctrl_RW 0x15
#define UB3_I2cSlv4Do_RW 0x16
#define UB3_I2cSlv4Di_R 0x17

/* AK09916 Magnetometer Definitions */
#define AK09916_I2C_ADDR 0x0C
#define AK09916_WIA2 0x01
#define AK09916_ST1 0x10
#define AK09916_HXL 0x11
#define AK09916_CNTL2 0x31
#define AK09916_CNTL3 0x32

/* I2C Master Control Bits */
#define I2C_MST_CLK_345_KHZ 0x07
#define I2C_MST_P_NSR 0x10 // STOP between reads

#endif /* __ICM20948_DEF_H */
