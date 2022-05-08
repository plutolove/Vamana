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
  DiskIndex(const std::string& path)
      : path(path), reader(path), head(std::make_shared<Block>(0, BLOK_SIZE)) {
    // index 最开始保存配置信息
    if (not reader.read(head)) {
      throw Exception(-1, "read head from {} failed, block id: 0", path);
    }
    size_t offset = 0;
    memcpy(&N, head->data, sizeof(N));
    offset += sizeof(N);
    memcpy(&dim, head->data + offset, sizeof(dim));
    offset += sizeof(dim);
    memcpy(&R, head->data + offset, sizeof(R));
    offset += sizeof(R);
    memcpy(&centroid_idx, head->data + offset, sizeof(centroid_idx));
    num_per_block = BLOK_SIZE / (sizeof(T) * dim + sizeof(int32_t) * (R + 1));
    std::cout << fmt::format(
        "N: {}, dim: {}, R:{}, centroid_idx: {}, num_per_block: {}\n", N, dim,
        R, centroid_idx, num_per_block);
  }

  inline int32_t block_id(size_t idx) { return idx / num_per_block + 1; }

 protected:
  std::string path;
  BlockReader reader;
  std::shared_ptr<Block> head;
  size_t num_per_block;
  size_t N, dim, R, centroid_idx;
  DistanceL2<T> calc;
};

}  // namespace vamana