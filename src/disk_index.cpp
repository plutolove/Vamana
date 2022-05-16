#include "block_pool.h"
#include "common/define.h"
#include "disk_index.h"

namespace vamana {

template <typename T>
DiskIndex<T>::DiskIndex(const std::string& path) : path(path), reader(path) {
  head = BlockPool::getInstance().getSingleBlockPtr();
  head->len = BLOK_SIZE;
  head->start = 0;
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
      "N: {}, dim: {}, R:{}, centroid_idx: {}, num_per_block: {}\n", N, dim, R,
      centroid_idx, num_per_block);
  size_per_record = sizeof(T) * dim + sizeof(int32_t) * (R + 1);
}

template class DiskIndex<float>;

}  // namespace vamana