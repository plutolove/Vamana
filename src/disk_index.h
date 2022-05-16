#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "block.h"
#include "boost/noncopyable.hpp"
#include "common/define.h"
#include "common/exception.h"
#include "distance.h"

namespace vamana {

template <typename T>
class DiskIndex : boost::noncopyable {
 public:
  DiskIndex(const std::string& path);

  inline int32_t block_id(size_t idx) { return idx / num_per_block + 1; }

 protected:
  std::string path;
  BlockReader reader;
  BlockPtr head;
  size_t num_per_block;
  size_t N, dim, R, centroid_idx;
  size_t size_per_record;
  DistanceL2<T> calc;
};

extern template class DiskIndex<float>;

}  // namespace vamana