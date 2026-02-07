#ifndef __DEVICE_CALIBRATION_H
#define __DEVICE_CALIBRATION_H

typedef struct {
  // 最终参数 (应用到算法中的)
  float offset[3]; // X, Y, Z 的偏置
  float scale[3];  // X, Y, Z 的比例系数 (通常接近 1.0)

  // 校准过程中的中间变量
  float min_val[3];  // 记录录得的最小值
  float max_val[3];  // 记录录得的最大值
  int sample_count;  // 采样计数
  int is_calibrated; // 标志位
} MagCalib_t;

// 初始化函数
void Mag_Calib_Init(MagCalib_t *calib);

// 输入新数据进行分析 (在校准模式下循环调用)
void Mag_Calib_Update(MagCalib_t *calib, float mx, float my, float mz);

// 计算最终结果 (校准结束时调用)
void Mag_Calib_Calc(MagCalib_t *calib);

// 应用校准参数 (在正常的读取循环中调用)
// input: raw_mx, raw_my, raw_mz
// output: calib_mx, calib_my, calib_mz
void Mag_Apply_Calib(MagCalib_t *calib, float raw_mx, float raw_my,
                     float raw_mz, float *out_mx, float *out_my, float *out_mz);

#endif // __DEVICE_CALIBRATION_H