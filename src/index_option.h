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
  float alpha = 1.2;
  std::string file_name;
  DistCalc calc;
};

}  // namespace vamana