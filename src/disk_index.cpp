#include <fmt/core.h>

#include <cstdint>
#include <queue>
#include <set>
#include <unordered_set>
#include <utility>

#include "block.h"
#include "block_cache.h"
#include "block_pool.h"
#include "common/define.h"
#include "disk_index.h"

namespace vamana {

template <typename T>
DiskIndex<T>::DiskIndex(const std::string& path, size_t cache_shard_num,
                        size_t cap)
    : path(path),
      reader(path),
      clock_cache(std::make_shared<SharedBlockCache>(cache_shard_num, cap)) {
  head = BlockPool::getInstance().getSingleBlockPtr();
  head->len = BLOCK_SIZE;
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
  num_per_block = BLOCK_SIZE / (sizeof(T) * dim + sizeof(int32_t) * (R + 1));
  size_per_record = sizeof(T) * dim + sizeof(int32_t) * (R + 1);
  std::cout << fmt::format(
      "N: {}, dim: {}, R:{}, centroid_idx: {}, num_per_block: {}, size per "
      "record: {}\n",
      N, dim, R, centroid_idx, num_per_block, size_per_record);

  // 起点开始3跳的block cache到static_cache
  init_static_cache();
}
template <typename T>
void DiskIndex<T>::init_static_cache() {
  std::queue<std::pair<int32_t, size_t>> q;
  std::unordered_set<int32_t> visit{int32_t(centroid_idx)};
  q.push(std::make_pair(int32_t(centroid_idx), 0));
  while (not q.empty()) {
    auto head = q.front();
    q.pop();

    // 只保存三跳的block
    if (head.second >= 3) continue;

    auto iter = static_cache.find(head.first);
    BlockPtr cur_block = nullptr;
    if (iter == static_cache.end()) {
      // 没有在cache中，则新建一个
      static_block.emplace_back();
      cur_block = &static_block.back();
      // 初始化block id，offset，len
      cur_block->idx = block_id(head.first);
      cur_block->start = block_offset(head.first);

      cur_block->len = BLOCK_SIZE;
      if (reader.read(cur_block)) {
        // 存到cache
        static_cache[cur_block->idx] = cur_block;
      }
    } else {
      cur_block = iter->second;
    }
    float* ptr = cur_block->getPtr<float>(vec_offset(head.first));

    int32_t num_neighbors =
        cur_block->getNeighborSize(num_neighbors_offset(head.first));

    int32_t* neighbors =
        cur_block->getPtr<int32_t>(neighbors_offset(head.first));

    for (int32_t i = 0; i < num_neighbors; i++) {
      auto it = static_cache.find(neighbors[i]);
      if (visit.count(neighbors[i])) continue;
      visit.insert(neighbors[i]);
      q.push(std::make_pair(neighbors[i], head.second + 1));
    }
  }
  std::cout << fmt::format("visit node size: {}, cache block size: {}\n",
                           visit.size(), static_cache.size());
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