#pragma once
#include <cstdint>
namespace vamana {

template <typename T>
struct Record {
  Record() {}
  Record(T* ptr, int32_t num, int32_t* neigs)
      : vec_ptr(ptr), num_neig(num), neighbors(neigs) {}
  T* vec_ptr = nullptr;
  int32_t num_neig;
  int32_t* neighbors = nullptr;
};

}  // namespace vamana