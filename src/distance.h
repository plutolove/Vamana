#pragma once
#include <cstdint>
#include <type_traits>

#include "common/distance_util.h"
#include "common/exception.h"
#include "fmt/core.h"

namespace vamana {

template <typename T>
struct DistanceL2 {
  float operator()(const T* x, const T* y, size_t dim) {
    if constexpr (std::is_same_v<T, float>) {
      return ComputeL2Distance_AVX(x, y, dim);
    } else if constexpr (std::is_same_v<T, int8_t>) {
      return ComputeL2Distance_AVX(x, y, dim);
    } else {
      throw Exception(-1, "not support type: {}", typeid(T).name());
    }
  }
};

}  // namespace vamana