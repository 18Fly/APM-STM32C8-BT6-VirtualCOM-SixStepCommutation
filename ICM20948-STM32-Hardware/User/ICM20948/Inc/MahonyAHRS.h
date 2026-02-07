#ifndef __MAHONY_AHRS_H
#define __MAHONY_AHRS_H

#include <math.h>

// 参数定义
// sampleFreq: 采样频率 (Hz)，例如 100.0f 或 200.0f
// twoKp: Proportional gain
// twoKi: Integral gain
#define DEFAULT_SAMPLE_FREQ 1100.0f
#define TWO_KP_DEF (5.0f)
#define TWO_KI_DEF (0.02f)

typedef struct {
  float q0, q1, q2, q3;                        // 四元数
  float integralFBx, integralFBy, integralFBz; // 积分误差累积
  float invSampleFreq;                         // 采样周期的倒数 (dt)
  float twoKp;                                 // 比例增益
  float twoKi;                                 // 积分增益
  float roll, pitch, yaw;                      // 欧拉角 (单位：度)
} Mahony_Handle_t;

// 初始化
void Mahony_Init(Mahony_Handle_t *filter, float sampleFreq);

// 9轴更新函数 (包含磁力计)
// gx, gy, gz: 陀螺仪数据 (单位：弧度/秒 rad/s) 注意单位
// ax, ay, az: 加速度计数据 (单位任意，算法内会归一化)
// mx, my, mz: 磁力计数据 (单位任意，算法内会归一化)
void Mahony_Update9DOF(Mahony_Handle_t *filter, float gx, float gy, float gz,
                       float ax, float ay, float az, float mx, float my,
                       float mz);

// 计算欧拉角
void Mahony_ComputeEuler(Mahony_Handle_t *filter);

#endif // __MAHONY_AHRS_H
