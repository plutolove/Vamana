#pragma once
#include <cstdlib>
#include <string>

namespace vamana {

template <typename DistCalc>
struct IndexOption {
  // search list size
  size_t L;
  // graph out degree
  size_t R;
  // vec dim
  size_t dim;
  // data size
  size_t N;
  //Centroid
  size_t centroid_idx;
  float alpha = 1.2;
  std::string file_name;
  std::string save_path;
  DistCalc calc;
};

}  // namespace vamana