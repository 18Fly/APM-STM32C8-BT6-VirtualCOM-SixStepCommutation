#include "MahonyAHRS.h"

// 快速平方根倒数
// (如果是STM32F4/F7等带FPU的芯片，直接用 1.0f/sqrtf()或者片上外挂FPU的使用提供的对应函数)
static float invSqrt(float x) { return 1.0f / sqrtf(x); }

void Mahony_Init(Mahony_Handle_t *filter, float sampleFreq) {
  filter->twoKp = TWO_KP_DEF;
  filter->twoKi = TWO_KI_DEF;
  filter->q0 = 1.0f;
  filter->q1 = 0.0f;
  filter->q2 = 0.0f;
  filter->q3 = 0.0f;
  filter->integralFBx = 0.0f;
  filter->integralFBy = 0.0f;
  filter->integralFBz = 0.0f;
  filter->invSampleFreq = 1.0f / sampleFreq;
}

void Mahony_Update9DOF(Mahony_Handle_t *filter, float gx, float gy, float gz,
                       float ax, float ay, float az, float mx, float my,
                       float mz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // 归一化加速度计
  recipNorm = invSqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // 归一化磁力计
  recipNorm = invSqrt(mx * mx + my * my + mz * mz);
  mx *= recipNorm;
  my *= recipNorm;
  mz *= recipNorm;

  // 辅助变量计算
  q0q0 = filter->q0 * filter->q0;
  q0q1 = filter->q0 * filter->q1;
  q0q2 = filter->q0 * filter->q2;
  q0q3 = filter->q0 * filter->q3;
  q1q1 = filter->q1 * filter->q1;
  q1q2 = filter->q1 * filter->q2;
  q1q3 = filter->q1 * filter->q3;
  q2q2 = filter->q2 * filter->q2;
  q2q3 = filter->q2 * filter->q3;
  q3q3 = filter->q3 * filter->q3;

  // 参考方向通量的计算 (Reference direction of flux)
  // 地磁场在机体坐标系下的估计
  hx = 2.0f *
       (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
  hy = 2.0f *
       (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
  bz = 2.0f *
       (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

  // 在水平面上的投影，计算地球磁场参考向量 b (bx, 0, bz)
  bx = sqrtf(hx * hx + hy * hy);

  // 估计重力方向和磁场方向 (Estimated direction of gravity and flux)
  // 重力向量估计 (v)
  halfvx = q1q3 - q0q2;
  halfvy = q0q1 + q2q3;
  halfvz = q0q0 - 0.5f + q3q3;

  // 磁场向量估计 (w)
  halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
  halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
  halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

  // 误差计算 (Error is sum of cross product between reference direction and
  // measured direction) 这里的 cross product 实际计算的是 2倍的误差
  halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
  halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
  halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

  // 积分误差缩放 (Compute and apply integral feedback if enabled)
  if (filter->twoKi > 0.0f) {
    filter->integralFBx += filter->twoKi * halfex * filter->invSampleFreq;
    filter->integralFBy += filter->twoKi * halfey * filter->invSampleFreq;
    filter->integralFBz += filter->twoKi * halfez * filter->invSampleFreq;
    gx += filter->integralFBx;
    gy += filter->integralFBy;
    gz += filter->integralFBz;
  } else {
    filter->integralFBx = 0.0f;
    filter->integralFBy = 0.0f;
    filter->integralFBz = 0.0f;
  }

  // 比例项校正 (Apply proportional feedback)
  gx += filter->twoKp * halfex;
  gy += filter->twoKp * halfey;
  gz += filter->twoKp * halfez;

  // 四元数微分方程积分 (Integrate rate of change of quaternion)
  gx *= (0.5f * filter->invSampleFreq);
  gy *= (0.5f * filter->invSampleFreq);
  gz *= (0.5f * filter->invSampleFreq);

  qa = filter->q0;
  qb = filter->q1;
  qc = filter->q2;

  filter->q0 += (-qb * gx - qc * gy - filter->q3 * gz);
  filter->q1 += (qa * gx + qc * gz - filter->q3 * gy);
  filter->q2 += (qa * gy - qb * gz + filter->q3 * gx);
  filter->q3 += (qa * gz + qb * gy - qc * gx);

  // 四元数归一化 (Normalise quaternion)
  recipNorm = invSqrt(filter->q0 * filter->q0 + filter->q1 * filter->q1 +
                      filter->q2 * filter->q2 + filter->q3 * filter->q3);
  filter->q0 *= recipNorm;
  filter->q1 *= recipNorm;
  filter->q2 *= recipNorm;
  filter->q3 *= recipNorm;
}

void Mahony_ComputeEuler(Mahony_Handle_t *filter) {
  // 根据具体定义的旋转顺序 (例如 Z-Y-X)，公式可能略有不同
  // 下面是常见的航空次序: Roll -> Pitch -> Yaw

  // Roll (x-axis rotation)
  float sinr_cosp = 2.0f * (filter->q0 * filter->q1 + filter->q2 * filter->q3);
  float cosr_cosp =
      1.0f - 2.0f * (filter->q1 * filter->q1 + filter->q2 * filter->q2);
  filter->roll = atan2f(sinr_cosp, cosr_cosp) * 57.29578f; // Rad to Deg

  // Pitch (y-axis rotation)
  float sinp = 2.0f * (filter->q0 * filter->q2 - filter->q3 * filter->q1);
  if (fabs(sinp) >= 1.0f)
    filter->pitch = copysignf(90.0f, sinp); // use 90 degrees if out of range
  else
    filter->pitch = asinf(sinp) * 57.29578f;

  // Yaw (z-axis rotation)
  float siny_cosp = 2.0f * (filter->q0 * filter->q3 + filter->q1 * filter->q2);
  float cosy_cosp =
      1.0f - 2.0f * (filter->q2 * filter->q2 + filter->q3 * filter->q3);
  filter->yaw = atan2f(siny_cosp, cosy_cosp) * 57.29578f;
}