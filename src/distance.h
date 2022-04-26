#pragma once
#include "common/distance_util.h"

namespace vamana {

struct DistanceL2Float {
  float operator()(const float* x, const float* y, size_t dim) {
    return ComputeL2Distance_AVX(x, y, dim);
  }
};

struct DistanceL2Int8 {
  float operator()(const int8_t* x, const int8_t* y, size_t dim) {
    return ComputeL2Distance_AVX(x, y, dim);
  }
};

}  // namespace vamana