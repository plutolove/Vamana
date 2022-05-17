#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "block.h"
#include "block_cache.h"
#include "boost/noncopyable.hpp"
#include "common/define.h"
#include "common/exception.h"
#include "distance.h"

namespace vamana {

template <typename T>
class DiskIndex : boost::noncopyable {
 public:
  struct Node {
    int32_t idx;
    float dist;
    std::vector<int32_t> neighbors;
    bool operator<(const Node& right) const { return dist > right.dist; }
  };

  DiskIndex(const std::string& path, size_t cache_shard_num, size_t cap);

  void init_static_cache();

  inline size_t block_id(size_t idx) { return idx / num_per_block + 1; }

  inline size_t block_offset(size_t idx) { return block_id(idx) * BLOCK_SIZE; }

  inline size_t vec_offset(size_t idx) {
    return idx % num_per_block * size_per_record;
  }

  inline size_t num_neighbors_offset(size_t idx) {
    return idx % num_per_block * size_per_record + sizeof(T) * dim;
  }

  inline size_t neighbors_offset(size_t idx) {
    return idx % num_per_block * size_per_record + sizeof(T) * dim +
           sizeof(int32_t);
  }

  std::vector<int32_t> search(T* query, size_t K, size_t L, size_t width);

 protected:
  std::string path;
  BlockReader reader;
  std::shared_ptr<SharedBlockCache> clock_cache;
  std::unordered_map<int32_t, BlockPtr> static_cache;
  std::list<Block> static_block;

  BlockPtr head;
  size_t num_per_block;
  size_t N, dim, R, centroid_idx;
  size_t size_per_record;
  DistanceL2<T> calc;
};

extern template class DiskIndex<float>;

}  // namespace vamana