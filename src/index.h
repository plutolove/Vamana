#pragma once
#include <array>

#include "boost/noncopyable.hpp"
#include "index_option.h"

namespace vamana {
template <typename T>
struct Entry {
  T* ptr;
};

template <typename T, typename DistCalc>
class VamanaIndex : boost::noncopyable {
 public:
  VamanaIndex(const IndexOption<DistCalc>& option) : option(option) {}

 protected:
  T* _data;
  IndexOption<DistCalc> option;
};

}  // namespace vamana