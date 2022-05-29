#include <fmt/core.h>
#include <libaio.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <ios>
#include <memory>
#include <queue>
#include <set>
#include <unordered_set>
#include <utility>

#include "block.h"
#include "block_cache.h"
#include "block_pool.h"
#include "common/define.h"
#include "disk_index.h"
#include "util/math_util.h"

namespace vamana {

template <typename T>
DiskIndex<T>::DiskIndex(const std::string& path,
                        const std::string& pq_index_path,
                        size_t cache_shard_num, size_t cap, size_t hop_num)
    : path(path),
      pq_index_path(pq_index_path),
      reader(path),
      clock_cache(std::make_shared<SharedBlockCache>(cache_shard_num, cap)),
      hop_num(hop_num),
      io_contexts(1024) {
  io_context_t ctx = 0;
  if (not io_contexts.pop(ctx)) {
    ctx = 0;
    int ret = io_setup(MAX_EVENTS, &ctx);
    if (ret != 0) {
      std::cout << fmt::format("io_setup() error, ret: {}, status: {}", ret,
                               strerror(ret))
                << std::endl;
    }
  }
  head = BlockPool::getInstance().getSingleBlockPtr();
  head->len = BLOCK_SIZE;
  head->start = 0;
  // index 最开始保存配置信息
  if (not reader.read(head, ctx)) {
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

  load_pq_index(pq_index_path);

  // 起点开始3跳的block cache到static_cache
  init_static_cache(hop_num, ctx);

  // 回收ctx
  if (not io_contexts.push(ctx)) {
    io_destroy(ctx);
  }
}

template <typename T>
void DiskIndex<T>::init_static_cache(size_t hop, io_context_t ctx) {
  std::queue<std::pair<int32_t, size_t>> q;
  std::unordered_set<int32_t> visit{int32_t(centroid_idx)};
  q.push(std::make_pair(int32_t(centroid_idx), 0));
  while (not q.empty()) {
    auto head = q.front();
    q.pop();
    auto block_idx = block_id(head.first);
    // 只保存hop的block
    if (head.second > hop) continue;

    auto iter = static_cache.find(block_idx);
    BlockPtr cur_block = nullptr;
    if (iter == static_cache.end()) {
      // 没有在cache中，则新建一个
      static_block.emplace_back();
      cur_block = &static_block.back();
      // 初始化block id，offset，len
      cur_block->idx = block_idx;
      cur_block->start = block_offset(head.first);

      cur_block->len = BLOCK_SIZE;
      if (reader.read(cur_block, ctx)) {
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

  io_context_t ctx = 0;
  if (not io_contexts.pop(ctx)) {
    ctx = 0;
    int ret = io_setup(MAX_EVENTS, &ctx);
    if (ret != 0) {
      std::cout << fmt::format("io_setup() error, ret: {}, status: {}", ret,
                               strerror(ret))
                << std::endl;
      return {};
    }
  }

  double read_tc = 0;

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
  size_t hit = 0;
  size_t not_hit = 0;
  while (not q.empty()) {
    Node top = q.top();
    q.pop();
    size_t idx = 0;
    while (idx < top.num_neighbors) {
      if (visit.count(top.neighbors[idx])) {
        idx++;
        continue;
      }

      std::vector<BlockPtr> uncached_blocks;
      std::vector<int32_t> uncached_idx;

      std::vector<int32_t> cached_idx;
      std::vector<int32_t> cached_blockid;
      while (uncached_blocks.size() < width && idx < top.num_neighbors) {
        auto neighbors_id = top.neighbors[idx];
        auto block_idx = block_id(neighbors_id);
        // 当前的id如果在之前用到的block中，则不再读取cache，直接从used_block中取
        auto it = used_block.find(block_idx);
        if (it != used_block.end()) {
          cached_idx.emplace_back(neighbors_id);
          cached_blockid.emplace_back(block_idx);
          idx++;
          hit++;
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
          hit++;
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
          hit++;
          continue;
        }
        // uncached_blocks
        auto new_block = block_pool.getSingleBlockPtr();
        new_block->idx = block_idx;
        new_block->start = block_offset(neighbors_id);
        new_block->len = BLOCK_SIZE;
        uncached_blocks.emplace_back(new_block);
        uncached_idx.emplace_back(neighbors_id);
        not_hit++;
        idx++;
      }
      auto st = std::chrono::high_resolution_clock::now();
      // async read uncached block
      reader.read(uncached_blocks, ctx);
      std::chrono::duration<double, std::milli> diff =
          std::chrono::high_resolution_clock::now() - st;
      read_tc += diff.count();

      // process cached
      for (size_t i = 0; i < cached_idx.size(); i++) {
        auto id = cached_idx[i];
        auto block_idx = cached_blockid[i];
        // 从used_block中获取对应的block
        auto* cur_block = used_block[block_idx];
        if (visit.count(id)) continue;
        auto* vec_ptr = cur_block->getPtr<float>(vec_offset(id));
        T dist = calc(query, vec_ptr, dim);
        if (topL.size() == L && dist >= topL.begin()->dist) continue;
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
          auto iter = topL.begin();
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
        if (topL.size() == L && dist >= topL.begin()->dist) continue;
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
          auto iter = topL.begin();
          topL.erase(*iter);
        }
      }
    }
  }

  // 回收ctx
  if (not io_contexts.push(ctx)) {
    io_destroy(ctx);
  }

  // 释放block cache
  for (auto& handle : handles) {
    clock_cache->release(handle);
  }
  ret.reserve(K);
  auto iter = topL.rbegin();
  while (K--) {
    if (iter == topL.rend()) break;
    ret.emplace_back(iter->idx);
    iter++;
  }
  std::cout << fmt::format("read block time cost: {}\n", read_tc);
  // std::cout << fmt::format("hit: {}, not_hit: {}\n", hit, not_hit);
  return ret;
}

template <typename T>
void DiskIndex<T>::load_pq_index(const std::string& pq_index_path) {
  std::ifstream fin(pq_index_path, std::ios_base::binary);
  size_t p_num;
  fin.read(reinterpret_cast<char*>(&p_num), sizeof(size_t));
  fin.read(reinterpret_cast<char*>(&M), sizeof(size_t));
  fin.read(reinterpret_cast<char*>(&sdim), sizeof(size_t));
  fin.read(reinterpret_cast<char*>(&cluster_num), sizeof(size_t));
  std::cout << fmt::format("N: {}, M: {}, sdim: {}, cluster_num: {}\n", p_num,
                           M, sdim, cluster_num);
  assert(N == p_num and N % M == 0 and dim / M == sdim);

  for (size_t id = 0; id < M; id++) {
    std::shared_ptr<T[]> cent_data(new T[cluster_num * sdim]);
    std::shared_ptr<T[]> pq(new T[cluster_num * cluster_num]);
    fin.read(reinterpret_cast<char*>(cent_data.get()),
             sizeof(T) * cluster_num * sdim);
    fin.read(reinterpret_cast<char*>(pq.get()),
             sizeof(T) * cluster_num * cluster_num);
    cluster_centers.emplace_back(cent_data);
    pq_dist.emplace_back(pq);
  }

  pq_code = std::make_unique<uint8_t[]>(N * M);

  for (size_t i = 0; i < N; i++) {
    fin.read(reinterpret_cast<char*>(pq_code.get() + i * M),
             M * sizeof(uint8_t));
  }

  fin.close();
}

template <typename T>
std::vector<int32_t> DiskIndex<T>::search_with_pq(T* query, size_t K, size_t L,
                                                  size_t width) {
  // block对象池，如果block不在cache中，从对象池分配block然后异步读
  static auto& block_pool = BlockPool::getInstance();

  io_context_t ctx = 0;
  if (not io_contexts.pop(ctx)) {
    ctx = 0;
    int ret = io_setup(MAX_EVENTS, &ctx);
    if (ret != 0) {
      std::cout << fmt::format("io_setup() error, ret: {}, status: {}", ret,
                               strerror(ret))
                << std::endl;
      return {};
    }
  }

  // 计算查询的点的pq code
  auto query_pq_code = compute_pq_code(query, cluster_centers, sdim);

  double read_tc = 0;

  std::vector<int32_t> ret;
  // 小根堆加速优化bfs搜索，每次取dist最小的node进行搜索
  std::priority_queue<Node> q;
  // 用于保存最后topL的结果，同时可以减枝
  // 距离大于topL biggest dist的node不再搜索
  std::set<Node> topL;
  std::set<Node> res;
  std::unordered_set<int32_t> visit;

  auto start_node = genStartNode(query);
  // 替换成pq距离
  start_node.dist =
      get_pq_dist(query_pq_code.data(), get_pq_code(centroid_idx), M);
  //初始化

  q.push(start_node);
  visit.insert(centroid_idx);

  // clock cache 的handle，用于后续释放
  std::vector<SharedBlockCache::CacheHandle*> handles;
  // 保存所有本次请求用到的block ptr
  std::unordered_map<int32_t, BlockPtr> used_block;
  size_t hit = 0;
  size_t not_hit = 0;
  while (not q.empty()) {
    std::vector<BlockPtr> uncached_blocks;
    std::vector<int32_t> uncached_idx;
    std::vector<int32_t> cached_idx;
    std::vector<int32_t> cached_blockid;

    size_t search_cnt = 0;
    // beam search，一次出队列最多width + 2个node
    // uncached block最多 width个
    while (not q.empty() and uncached_blocks.size() < width and
           search_cnt < width * 2) {
      Node top = q.top();
      search_cnt++;
      q.pop();
      auto neighbors_id = top.idx;
      auto block_idx = block_id(neighbors_id);
      // 当前的id如果在之前用到的block中，则不再读取cache，直接从used_block中取
      auto it = used_block.find(block_idx);
      if (it != used_block.end()) {
        cached_idx.emplace_back(neighbors_id);
        cached_blockid.emplace_back(block_idx);
        hit++;
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
        hit++;
        continue;
      }
      // block cache
      auto handle = clock_cache->find(block_idx);
      if (handle) {
        cached_idx.emplace_back(neighbors_id);
        cached_blockid.emplace_back(block_idx);
        used_block[block_idx] = handle->value;
        handles.emplace_back(handle);
        hit++;
        continue;
      }
      // uncached_blocks
      auto new_block = block_pool.getSingleBlockPtr();
      new_block->idx = block_idx;
      new_block->start = block_offset(neighbors_id);
      new_block->len = BLOCK_SIZE;
      uncached_blocks.emplace_back(new_block);
      uncached_idx.emplace_back(neighbors_id);
      not_hit++;
    }
    // async read uncached block
    if (not uncached_blocks.empty()) reader.read(uncached_blocks, ctx);

    // process cached block
    for (size_t i = 0; i < cached_idx.size(); i++) {
      auto id = cached_idx[i];
      auto block_idx = cached_blockid[i];
      // 从used_block中获取对应的block
      auto* cur_block = used_block[block_idx];
      auto* vec_ptr = cur_block->getPtr<float>(vec_offset(id));
      T dist = calc(query, vec_ptr, dim);
      // 最终结果用精确距离来判断
      Node v;
      v.idx = id;
      v.dist = dist;
      res.insert(v);
      if (res.size() > L) {
        auto iter = res.begin();
        res.erase(*iter);
      }

      // 获取该节点的邻居
      int32_t num_neighbors =
          cur_block->getNeighborSize(num_neighbors_offset(id));
      int32_t* neighbors = cur_block->getPtr<int32_t>(neighbors_offset(id));
      // 遍历所有邻居
      for (size_t j = 0; j < num_neighbors; j++) {
        // 访问过的跳过
        if (visit.count(neighbors[j])) continue;
        visit.insert(neighbors[j]);
        // 查表，得到pq code
        auto* nei_pq_code = get_pq_code(neighbors[j]);
        // 计算pq距离
        auto pq_dis = get_pq_dist(nei_pq_code, query_pq_code.data(), sdim);
        // 减枝
        if (topL.size() == L && pq_dis >= topL.begin()->dist) continue;
        Node nd;
        nd.dist = pq_dis;
        nd.dist = pq_dis;
        // 入队列
        q.push(nd);
        // 加入topL候选
        topL.insert(nd);
        if (topL.size() > L) {
          auto iter = topL.begin();
          topL.erase(*iter);
        }
      }
    }
    // process uncached block
    for (size_t i = 0; i < uncached_idx.size(); i++) {
      auto id = uncached_idx[i];
      // 从used_block中获取对应的block
      auto* cur_block = uncached_blocks[i];
      auto* vec_ptr = cur_block->getPtr<float>(vec_offset(id));
      T dist = calc(query, vec_ptr, dim);
      // 最终结果用精确距离来判断
      Node v;
      v.idx = id;
      v.dist = dist;
      res.insert(v);
      if (res.size() > L) {
        auto iter = res.begin();
        res.erase(*iter);
      }

      // 获取该节点的邻居
      int32_t num_neighbors =
          cur_block->getNeighborSize(num_neighbors_offset(id));
      int32_t* neighbors = cur_block->getPtr<int32_t>(neighbors_offset(id));
      // 遍历所有邻居
      for (size_t j = 0; j < num_neighbors; j++) {
        // 访问过的跳过
        if (visit.count(neighbors[j])) continue;
        visit.insert(neighbors[j]);
        // 查表，得到pq code
        auto* nei_pq_code = get_pq_code(neighbors[j]);
        // 计算pq距离
        auto pq_dis = get_pq_dist(nei_pq_code, query_pq_code.data(), sdim);
        // 减枝
        if (topL.size() == L && pq_dis >= topL.begin()->dist) continue;
        Node nd;
        nd.dist = pq_dis;
        nd.dist = pq_dis;
        // 入队列
        q.push(nd);
        // 加入topL候选
        topL.insert(nd);
        if (topL.size() > L) {
          auto iter = topL.begin();
          topL.erase(*iter);
        }
      }

      // insert cache，not success recycle block
      if (not clock_cache->insert(cur_block->idx, cur_block)) {
        block_pool.recycle(cur_block);
      }
    }
  }

  // 回收ctx
  if (not io_contexts.push(ctx)) {
    io_destroy(ctx);
  }

  // 释放block cache
  for (auto& handle : handles) {
    clock_cache->release(handle);
  }

  ret.reserve(K);

  auto iter = topL.rbegin();
  while (K--) {
    if (iter == topL.rend()) break;
    ret.emplace_back(iter->idx);
    iter++;
  }

  std::cout << fmt::format("read block time cost: {}\n", read_tc);
  // std::cout << fmt::format("hit: {}, not_hit: {}\n", hit, not_hit);
  return ret;
}

template class DiskIndex<float>;

}  // namespace vamana