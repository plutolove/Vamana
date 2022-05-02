#pragma once
#include <cstdlib>
#include <string>

#include "distance.h"
namespace vamana {

template <typename T>
struct IndexOption {
  // search list size
  size_t L;
  // graph out degree
  size_t R;
  // vec dim
  size_t dim;
  // data size
  size_t N;
  // max candiate
  size_t C = 750;
  // test data size
  size_t test_N;
  // thread num
  int thread_num = 16;
  std::string test_file;
  // Centroid
  size_t centroid_idx;
  float alpha = 1.2;
  std::string file_name;
  std::string save_path;
  DistanceL2<T> calc;
};

}  // namespace vamana