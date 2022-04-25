#pragma once
#include <cstdlib>
#include <string>

namespace vamana {

template <typename DistCalc>
struct IndexOption {
  size_t L;
  size_t R;
  size_t dim;
  float alpha;
  std::string file_name;
  DistCalc calc;
};

}  // namespace vamana