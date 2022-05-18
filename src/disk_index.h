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
    int32_t idx = -1;
    float dist = 0;
    size_t num_neighbors = 0;
    std::vector<int32_t> neighbors;
    Node() {}
    Node(int32_t idx, float dist, size_t num, int32_t* neig)
        : idx(idx), dist(dist), num_neighbors(num) {
      neighbors.resize(num_neighbors);
      std::memcpy(neighbors.data(), neig, sizeof(int32_t) * num_neighbors);
    }
    bool operator<(const Node& right) const { return dist > right.dist; }
  };

  DiskIndex(const std::string& path, size_t cache_shard_num, size_t cap);

  void init_static_cache(size_t hop);

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

  Node genStartNode(T* query) {
    // 质心的block id
    static size_t start_block_id = block_id(centroid_idx);
    // 质心的block肯定在static cache中
    static BlockPtr start_block_ptr = static_cache[start_block_id];
    // 质心对应的vec ptr
    static auto* start_vec_ptr =
        start_block_ptr->getPtr<float>(vec_offset(centroid_idx));
    // 质心的num neighbors
    static int32_t start_num_neighbors =
        start_block_ptr->getNeighborSize(num_neighbors_offset(centroid_idx));
    // 质心的neighbors
    static int32_t* start_neighbors =
        start_block_ptr->getPtr<int32_t>(neighbors_offset(start_block_id));
    // 质心到query的距离
    float dist = calc(query, start_vec_ptr, dim);
    Node nd(centroid_idx, dist, start_num_neighbors, start_neighbors);
    return nd;
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