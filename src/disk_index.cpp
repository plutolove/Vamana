#include <fmt/core.h>
#include <libaio.h>

#include <cstddef>
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
  init_static_cache(3);
}

template <typename T>
void DiskIndex<T>::init_static_cache(size_t hop) {
  std::queue<std::pair<int32_t, size_t>> q;
  std::unordered_set<int32_t> visit{int32_t(centroid_idx)};
  q.push(std::make_pair(int32_t(centroid_idx), 0));
  while (not q.empty()) {
    auto head = q.front();
    q.pop();

    // 只保存hop的block
    if (head.second > hop) continue;

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
      if (visit.count(neighbors[i]) || head.second + 1 > hop) continue;
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
  // block对象池，如果block不在cache中，从对象池分配block然后异步读
  static auto& block_pool = BlockPool::getInstance();

  std::vector<int32_t> ret;
  // 小根堆加速优化bfs搜索，每次取dist最小的node进行搜索
  std::priority_queue<Node> q;
  // 用于保存最后topL的结果，同时可以减枝
  // 距离大于topL biggest dist的node不再搜索
  std::set<Node> topL;
  std::unordered_set<int32_t> visit;

  //初始化
  q.push(genStartNode(query));
  visit.insert(centroid_idx);

  // clock cache 的handle，用于后续释放
  std::vector<SharedBlockCache::CacheHandle*> handles;
  // 保存所有本次请求用到的block ptr
  std::unordered_map<int32_t, BlockPtr> used_block;

  while (not q.empty()) {
    Node top = q.top();
    q.pop();
    size_t idx = 0;
    while (idx < top.num_neighbors) {
      auto neighbors_id = top.neighbors[idx];
      if (visit.count(neighbors_id)) {
        idx++;
        continue;
      }

      std::vector<BlockPtr> uncached_blocks;
      std::vector<int32_t> uncached_idx;

      std::vector<int32_t> cached_idx;
      std::vector<int32_t> cached_blockid;
      while (uncached_blocks.size() < width && idx < top.num_neighbors) {
        auto block_idx = block_id(neighbors_id);
        // 当前的id如果在之前用到的block中，则不再读取cache，直接从used_block中取
        auto it = used_block.find(block_idx);
        if (it != used_block.end()) {
          cached_idx.emplace_back(neighbors_id);
          cached_blockid.emplace_back(block_idx);
          idx++;
          continue;
        }
        // if block cache get block
        // cached_blocks push_back
        // 3跳的cache
        auto iter = static_cache.find(block_idx);
        if (iter != static_cache.end()) {
          cached_idx.emplace_back(neighbors_id);
          used_block[block_idx] = iter->second;
          cached_blockid.emplace_back(block_idx);
          idx++;
          continue;
        }
        // block cache
        auto handle = clock_cache->find(block_idx);
        if (handle) {
          cached_idx.emplace_back(neighbors_id);
          cached_blockid.emplace_back(block_idx);
          used_block[block_idx] = handle->value;
          handles.emplace_back(handle);
          idx++;
          continue;
        }
        // uncached_blocks
        auto new_block = block_pool.getSingleBlockPtr();
        new_block->idx = block_idx;
        new_block->start = block_offset(neighbors_id);
        new_block->len = BLOCK_SIZE;
        uncached_blocks.emplace_back(new_block);
        uncached_idx.emplace_back(neighbors_id);
        idx++;
      }
      // async read uncached block
      reader.read(uncached_blocks);
      // process cached
      for (size_t i = 0; i < cached_idx.size(); i++) {
        auto id = cached_idx[i];
        auto block_idx = cached_blockid[i];
        // 从used_block中获取对应的block
        auto* cur_block = used_block[block_idx];
        if (visit.count(id)) continue;
        auto* vec_ptr = cur_block->getPtr<float>(vec_offset(id));
        T dist = calc(query, vec_ptr, dim);
        if (topL.size() == L && dist >= topL.rbegin()->dist) continue;
        int32_t num_neighbors =
            cur_block->getNeighborSize(num_neighbors_offset(id));

        int32_t* neighbors = cur_block->getPtr<int32_t>(neighbors_offset(id));
        // 构造node
        Node nd(id, dist, num_neighbors, neighbors);

        //入队列
        q.push(nd);
        topL.insert(nd);
        // 标记访问过
        visit.insert(id);
        if (topL.size() > L) {
          auto iter = topL.rbegin();
          topL.erase(*iter);
        }
      }

      // process uncached block
      // 一次最多有width个uncached block要处理
      for (size_t i = 0; i < uncached_idx.size(); i++) {
        auto id = uncached_idx[i];
        auto* cur_block = uncached_blocks[i];
        if (visit.count(id)) continue;
        auto* vec_ptr = cur_block->getPtr<float>(vec_offset(id));
        T dist = calc(query, vec_ptr, dim);
        // 距离大于等于topL中最大的则跳过
        if (topL.size() == L && dist >= topL.rbegin()->dist) continue;
        int32_t num_neighbors =
            cur_block->getNeighborSize(num_neighbors_offset(id));

        int32_t* neighbors = cur_block->getPtr<int32_t>(neighbors_offset(id));
        // 构造node
        Node nd(id, dist, num_neighbors, neighbors);

        q.push(nd);
        topL.insert(nd);
        // 标记访问过
        visit.insert(id);
        // insert 到block cache，如果insert cache失败则直接回收block
        auto* hd = clock_cache->insert_and_hold(cur_block->idx, cur_block);
        if (hd) {
          used_block[cur_block->idx] = cur_block;
          handles.emplace_back(hd);
        } else {
          block_pool.recycle(cur_block);
        }
        if (topL.size() > L) {
          auto iter = topL.rbegin();
          topL.erase(*iter);
        }
      }
    }
  }
  // 释放block cache
  for (auto& handle : handles) {
    clock_cache->release(handle);
  }
  ret.reserve(K);
  auto iter = topL.begin();
  while (K--) {
    if (iter == topL.end()) break;
    ret.emplace_back(iter->idx);
    iter++;
  }
  return ret;
}

template class DiskIndex<float>;

}  // namespace vamana