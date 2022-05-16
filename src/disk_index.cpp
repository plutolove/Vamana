#include <cstdint>
#include <queue>
#include <set>
#include <unordered_set>

#include "block.h"
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

template <typename T>
std::vector<int32_t> DiskIndex<T>::search(T* query, size_t K, size_t L,
                                          size_t width) {
  std::vector<int32_t> ret;
  std::priority_queue<Node> q;
  std::set<Node> topL;
  std::unordered_set<int32_t> visit;

  auto& block_pool = BlockPool::getInstance();

  while (not q.empty()) {
    Node top = q.top();
    q.pop();
    size_t idx = 0;
    while (idx < top.neighbors.size()) {
      if (visit.count(top.neighbors[idx])) continue;

      std::vector<BlockPtr> uncached_blocks;
      std::vector<int32_t> uncached_idx;

      std::vector<BlockPtr> cached_blocks;
      std::vector<int32_t> cached_idx;
      while (uncached_blocks.size() < width && idx < top.neighbors.size()) {
        // if block cache get block
        // cached_blocks push_back

        // else
        // uncached_blocks push_back
        idx++;
      }
      // async read uncached block
      reader.read(uncached_blocks);
      // process cached
      // process uncached block

      // if (topL.size() == L &&)
    }
  }
  return ret;
}

template class DiskIndex<float>;

}  // namespace vamana