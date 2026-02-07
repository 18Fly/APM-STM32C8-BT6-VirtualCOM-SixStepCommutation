#include "DeviceCalibration.h"
#include <float.h> // 用于 FLT_MAX
#include <math.h>

void Mag_Calib_Init(MagCalib_t *calib) {
  // 初始化 Min 为最大浮点数，Max 为最小浮点数
  for (int i = 0; i < 3; i++) {
    calib->min_val[i] = FLT_MAX;
    calib->max_val[i] = -FLT_MAX;
    calib->offset[i] = 0.0f;
    calib->scale[i] = 1.0f;
  }
  calib->sample_count = 0;
  calib->is_calibrated = 0;
}

void Mag_Calib_Update(MagCalib_t *calib, float mx, float my, float mz) {
  float val[3] = {mx, my, mz};

  // 简单的极值记录
  for (int i = 0; i < 3; i++) {
    if (val[i] < calib->min_val[i]) {
      calib->min_val[i] = val[i];
    }
    if (val[i] > calib->max_val[i]) {
      calib->max_val[i] = val[i];
    }
  }
  calib->sample_count++;
}

void Mag_Calib_Calc(MagCalib_t *calib) {
  float delta[3];
  float avg_delta = 0.0f;

  // 计算 Offset (球心)
  // Offset = (Max + Min) / 2
  for (int i = 0; i < 3; i++) {
    calib->offset[i] = (calib->max_val[i] + calib->min_val[i]) / 2.0f;

    // 计算轴跨度 (Max - Min)
    delta[i] = (calib->max_val[i] - calib->min_val[i]);
    avg_delta += delta[i];
  }

  avg_delta /= 3.0f; // 算出三个轴平均的“直径”

  // 计算 Scale (软铁校准)
  // 目标是让所有轴的“直径”都等于 avg_delta
  // Scale_X = Avg_Diameter / Diameter_X
  for (int i = 0; i < 3; i++) {
    if (delta[i] != 0) {
      calib->scale[i] = avg_delta / delta[i];
    } else {
      calib->scale[i] = 1.0f; // 防止除以0
    }
  }

  calib->is_calibrated = 1;
}

void Mag_Apply_Calib(MagCalib_t *calib, float raw_mx, float raw_my,
                     float raw_mz, float *out_mx, float *out_my,
                     float *out_mz) {

  // 公式: Out = (Raw - Offset) * Scale
  *out_mx = (raw_mx - calib->offset[0]) * calib->scale[0];
  *out_my = (raw_my - calib->offset[1]) * calib->scale[1];
  *out_mz = (raw_mz - calib->offset[2]) * calib->scale[2];
}
