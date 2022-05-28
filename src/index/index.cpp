#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <ios>
#include <memory>
#include <mutex>
#include <random>
#include <utility>
#include <vector>

#include "common/define.h"
#include "distance.h"
#include "fmt/format.h"
#include "index.h"
#include "omp.h"
#include "util/math_util.h"

namespace vamana {

template <typename T>
size_t VamanaIndex<T>::calcCentroid() {
  std::vector<T> ce(option.dim, 0);
  // 初始化质心
  for (size_t i = 0; i < option.N; ++i) {
    for (size_t j = 0; j < option.dim; ++j) {
      ce[j] += vec_ptr[i][j];
    }
  }
  for (size_t i = 0; i < option.dim; ++i) ce[i] /= option.N;

  std::vector<T> dist(option.N, 0);

  const auto* ce_ptr = ce.data();
#pragma omp parallel for schedule(dynamic, 65536)
  for (size_t i = 0; i < option.N; ++i) {
    dist[i] = option.calc(vec_ptr[i], ce_ptr, option.dim);
  }

  size_t idx = 0;
  for (size_t i = 0; i < option.N; i++) {
    if (dist[i] < dist[idx]) {
      idx = i;
    }
  }
  std::cout << fmt::format("calcCentroid idx: {}, dist: {}\n", idx, dist[idx]);
  option.centroid_idx = idx;
  return idx;
}

template <typename T>
size_t VamanaIndex<T>::bfsSearch(
    size_t s, const T* q, size_t k, size_t L,
    std::vector<std::pair<T, size_t>>& visited_node) {
  // 遍历过的点
  std::unordered_set<size_t> visit{s};
  std::set<PI> V, topL;

  V.insert({option.calc(q, vec_ptr[s], option.dim), s});
  topL.insert({option.calc(q, vec_ptr[s], option.dim), s});

  auto s_dist = option.calc(q, vec_ptr[s], option.dim);

  // 小根堆保存dist和idx
  std::priority_queue<PI, std::deque<PI>, std::greater<PI>> query;
  // std::priority_queue<> query;
  query.push(std::make_pair(option.calc(vec_ptr[s], q, option.dim), s));
  // bfs搜索
  while (not query.empty()) {
    auto front = query.top();
    query.pop();
    auto& children = _graph[front.second];
    for (auto child : children) {
      T cur_dist = option.calc(q, vec_ptr[child], option.dim);
      // 访问过的不再访问
      if (visit.count(child)) continue;
      // 距离大于目前搜索最优的L个点则不搜索该点
      if (topL.size() == L && cur_dist >= topL.rbegin()->first) {
        continue;
      }

      PI kv = std::make_pair(cur_dist, child);
      // 入队列
      query.push(kv);

      // 标记遍历过了
      visit.insert(child);
      // 直接返回访问过的点和距离，后续可以直接使用，不用重复计算
      V.insert(kv);
      // 保存到topL结果中
      topL.insert(kv);
      // 删除size>L的距离最大的点
      if (topL.size() > L) {
        auto iter = topL.rbegin();
        topL.erase(*iter);
      }
    }
  }

  visited_node.reserve(V.size());
  for (auto& kv : V) {
    visited_node.push_back(kv);
  }

  return topL.begin()->second;
}

template <typename T>
void VamanaIndex<T>::build() {
  // 分NUM_SYNCS次batch执行
  size_t NUM_SYNCS = DIV_ROUND_UP(option.N, (64 * 64));
  size_t round_size = DIV_ROUND_UP(option.N, NUM_SYNCS);  // size of each batch
  if (option.thread_num > 0) omp_set_num_threads(option.thread_num);
  init_random_graph();
  std::vector<size_t> index_data(option.N);
  std::iota(index_data.begin(), index_data.end(), 0);
  // 随机打散所有点，用于随机遍历所有点，构建索引
  std::random_shuffle(index_data.begin(), index_data.end());

  size_t ep_idx = calcCentroid();
  constexpr size_t ITER_NUM = 2;
  float alpha = option.alpha;
  for (size_t iter_id = 0; iter_id < ITER_NUM; ++iter_id) {
    auto s = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff;

    if (iter_id == (ITER_NUM - 1)) {
      option.alpha = alpha;
    } else {
      option.alpha = 1;
    }
    std::cout << "alpha: " << option.alpha << std::endl;
    // 标记需要异步更新图的点
    std::vector<bool> need_to_sync(option.N, false);
    // 要调整图的节点，分round_size，可以并行计算
    std::vector<std::vector<size_t>> pruned_list_vec(round_size);

    for (size_t sync_num = 0; sync_num < NUM_SYNCS; sync_num++) {
      size_t start_id = sync_num * round_size;
      size_t end_id = std::min(option.N, (sync_num + 1) * round_size);
      // 这一步拿到所有节点的要更新的链接点
#pragma omp parallel for schedule(dynamic)
      for (size_t idx = start_id; idx < end_id; ++idx) {
        size_t node_idx = index_data[idx];
        size_t node_offset = idx - start_id;
        // topk和visit都是按distance从小到达排好顺序的
        std::vector<std::pair<T, size_t>> visit;
        std::set<size_t> visit_idx;
        auto& pruned_vec = pruned_list_vec[node_offset];
        auto ridx = bfsSearch(ep_idx, vec_ptr[node_idx], 1, option.L, visit);
        for (auto& kv : visit) {
          visit_idx.insert(kv.second);
        }
        for (auto v : _graph[node_idx]) {
          if (v != node_idx && visit_idx.find(v) == visit_idx.end()) {
            visit.push_back(std::make_pair(
                option.calc(vec_ptr[node_idx], vec_ptr[v], option.dim), v));
            visit_idx.insert(v);
          }
        }
        prune_neighbors(node_idx, option.R, option.C, option.alpha, visit,
                        pruned_vec);
        // std::cout << fmt::format(
        //     "bfs search running, thread num: {}, thread id: {}, topk size:
        //     {}, " "visit size: {}\n",
        //    omp_get_num_threads(), omp_get_thread_num(), topk.size(),
        //   visit.size());
      }

      // 通过上一步拿到的要更新的链接点更新
#pragma omp parallel for schedule(dynamic, 64)
      for (size_t idx = start_id; idx < end_id; ++idx) {
        size_t node_idx = index_data[idx];
        size_t node_offset = idx - start_id;
        std::vector<size_t>& pruned_vec = pruned_list_vec[node_offset];
        _graph[node_idx].clear();
        for (auto id : pruned_vec) _graph[node_idx].emplace_back(id);
      }

#pragma omp parallel for schedule(dynamic, 64)
      for (size_t idx = start_id; idx < end_id; ++idx) {
        size_t node_idx = index_data[idx];
        size_t node_offset = idx - start_id;
        std::vector<size_t>& pruned_vec = pruned_list_vec[node_offset];
        batch_update(node_idx, pruned_vec, option.R, need_to_sync);
        pruned_vec.clear();
      }
#pragma omp parallel for schedule(dynamic, 65536)
      for (size_t idx = 0; idx < index_data.size(); ++idx) {
        size_t node_idx = index_data[idx];
        if (need_to_sync[node_idx]) {
          need_to_sync[node_idx] = false;
          std::unordered_set<size_t> visit_idx;
          std::vector<PI> visit;
          std::vector<size_t> new_out;
          for (auto& v : _graph[node_idx]) {
            if (visit_idx.find(v) == visit_idx.end() && v != node_idx) {
              float dist =
                  option.calc(vec_ptr[node_idx], vec_ptr[v], option.dim);
              visit.emplace_back(std::make_pair(dist, v));
              visit_idx.insert(v);
            }
          }
          prune_neighbors(node_idx, option.R, option.C, option.alpha, visit,
                          new_out);
          _graph[node_idx].clear();
          for (auto id : new_out) _graph[node_idx].emplace_back(id);
        }
      }
    }
    diff = std::chrono::high_resolution_clock::now() - s;
    std::cout << fmt::format("iter_id: {}, alpha: {}, time cost: {}s\n",
                             iter_id, option.alpha, diff.count());
  }
#pragma omp parallel for schedule(dynamic, 65536)
  for (size_t idx = 0; idx < index_data.size(); ++idx) {
    size_t node_idx = index_data[idx];
    if (_graph[node_idx].size() > option.R) {
      std::unordered_set<size_t> visit_idx;
      std::vector<PI> visit;
      std::vector<size_t> new_out;
      for (auto& v : _graph[node_idx]) {
        if (visit_idx.find(v) == visit_idx.end() && v != node_idx) {
          float dist = option.calc(vec_ptr[node_idx], vec_ptr[v], option.dim);
          visit.emplace_back(std::make_pair(dist, v));
          visit_idx.insert(v);
        }
      }
      prune_neighbors(node_idx, option.R, option.C, option.alpha, visit,
                      new_out);
      _graph[node_idx].clear();
      for (auto id : new_out) _graph[node_idx].emplace_back(id);
    }
  }
}

template <typename T>
void VamanaIndex<T>::batch_update(size_t node_idx,
                                  const std::vector<size_t>& pruned_list,
                                  size_t R, std::vector<bool>& need_to_sync) {
  for (auto v : pruned_list) {
    if (v == node_idx) continue;

    assert(v >= 0 && v < option.N);
    {
      // 将node_idx和pruned_list中的节点建立链接
      // 达到阈值才批量pruned一次
      std::lock_guard guard(_locks[v]);
      if (std::find(_graph[v].begin(), _graph[v].end(), node_idx) ==
          _graph[v].end()) {
        _graph[v].push_back(node_idx);
        if (_graph[v].size() >= (size_t)(R * 1.2)) need_to_sync[v] = true;
      }
    }
  }
}

template <typename T>
void VamanaIndex<T>::occlude_list(std::vector<PI>& node_list, float alpha,
                                  size_t degree, size_t maxc,
                                  std::vector<PI>& result) {
  std::vector<float> occlude_factor(node_list.size(), 0);
  occlude_list(node_list, alpha, degree, maxc, result, occlude_factor);
}

template <typename T>
void VamanaIndex<T>::occlude_list(std::vector<PI>& node_list, float alpha,
                                  size_t degree, size_t maxc,
                                  std::vector<PI>& result,
                                  std::vector<float>& occlude_factor) {
  if (node_list.empty()) return;
  float cur_alpha = 1;
  while (cur_alpha <= alpha && result.size() < degree) {
    size_t start = 0;
    while (result.size() < degree && start < node_list.size() && start < maxc) {
      auto& p = node_list[start];
      if (occlude_factor[start] > cur_alpha) {
        start++;
        continue;
      }
      // 表示该点取过了
      occlude_factor[start] = std::numeric_limits<float>::max();
      result.push_back(p);
      for (size_t t = start + 1; t < node_list.size() && t < maxc; t++) {
        // alpha * dist(p*, p') < dist(p, p')  -> remove(p')
        if (occlude_factor[t] > alpha) continue;
        float dist = option.calc(vec_ptr[p.second],
                                 vec_ptr[node_list[t].second], option.dim);

        occlude_factor[t] =
            std::max(occlude_factor[t], node_list[t].first / dist);
      }
      start++;
    }
    cur_alpha *= 1.2;
  }
}

template <typename T>
void VamanaIndex<T>::prune_neighbors(size_t node_idx, size_t R, size_t C,
                                     float alpha, std::vector<PI>& node_list,
                                     std::vector<size_t>& pruned_list) {
  if (node_list.empty()) return;
  // 按dist排序
  std::sort(node_list.begin(), node_list.end());
  std::vector<PI> result;
  result.reserve(R);
  std::vector<float> occlude_factor(node_list.size(), 0);
  occlude_list(node_list, alpha, R, C, result, occlude_factor);
  pruned_list.clear();
  assert(result.size() <= R);
  for (auto iter : result) {
    if (iter.second != node_idx) pruned_list.emplace_back(iter.second);
  }
}

template <typename T>
size_t VamanaIndex<T>::save_disk_index(const std::string& path) {
  // disk index format
  /*
  first block: the number of point(size_t), dim(size_t), centroid_idx(size_t)

  4k per block: [vec: the number of neighbors: neighbors]
  */
  std::ofstream fout(path, std::ios_base::binary);
  // 4k per block
  char block[BLOCK_SIZE];
  int32_t neighbors[200];
  // write the number of point(size_t), dim(size_t), centroid_idx(size_t)
  int offset = 0;
  memset(block, -1, sizeof(block));

  memcpy(block, &option.N, sizeof(option.N));
  offset += sizeof(option.N);

  memcpy(block + offset, &option.dim, sizeof(option.dim));
  offset += sizeof(option.dim);

  memcpy(block + offset, &option.R, sizeof(option.R));
  offset += sizeof(option.R);

  memcpy(block + offset, &option.centroid_idx, sizeof(option.centroid_idx));

  fout.write(block, sizeof(char) * BLOCK_SIZE);

  size_t num_per_block =
      BLOCK_SIZE / (sizeof(T) * option.dim + sizeof(int32_t) * (option.R + 1));

  size_t block_num = ROUND_UP(option.N, num_per_block);
  size_t idx = 0;
  for (size_t block_id = 0; block_id < block_num; ++block_id) {
    // block 全初始化-1
    memset(block, -1, sizeof(block));
    std::string raw_data;
    for (size_t id = 0; id < num_per_block and idx < option.N; ++id) {
      // 将embedding保存到raw_data
      raw_data.append(reinterpret_cast<const char*>(vec_ptr[idx]),
                      sizeof(T) * option.dim);
      int32_t num_neighbors = _graph[idx].size();
      raw_data.append(reinterpret_cast<char*>(&num_neighbors), sizeof(int32_t));
      // neighbors 按R保存，不够的补-1
      memset(neighbors, -1, sizeof(neighbors));
      size_t neighbors_idx = 0;
      for (auto& v : _graph[idx]) {
        neighbors[neighbors_idx++] = v;
      }
      // 保存neighbors到raw_data
      raw_data.append(reinterpret_cast<char*>(neighbors),
                      sizeof(int32_t) * option.R);

      idx++;
    }
    memcpy(block, raw_data.data(), raw_data.size());
    // 一整个block写入
    fout.write(block, BLOCK_SIZE);
  }
  fout.close();
  return 0;
}

template <typename T>
size_t VamanaIndex<T>::load_disk_index(const std::string& path) {
  std::ifstream fin(path, std::ios_base::binary);

  char block[BLOCK_SIZE];
  fin.read(block, BLOCK_SIZE);
  size_t N, dim, R, centroid_idx;
  size_t offset = 0;
  memcpy(&N, block, sizeof(N));
  offset += sizeof(N);
  memcpy(&dim, block + offset, sizeof(dim));
  offset += sizeof(dim);
  memcpy(&R, block + offset, sizeof(R));
  offset += sizeof(R);
  memcpy(&centroid_idx, block + offset, sizeof(centroid_idx));
  std::cout << fmt::format("n:{}, dim:{}, r:{}, idx:{}\n", N, dim, R,
                           centroid_idx);
  size_t num_per_block =
      BLOCK_SIZE / (sizeof(T) * option.dim + sizeof(int32_t) * (option.R + 1));

  size_t block_num = ROUND_UP(option.N, num_per_block);
  size_t idx = 0;

  for (size_t block_id = 0; block_id < block_num; ++block_id) {
    fin.read(block, BLOCK_SIZE);
    auto* ptr = reinterpret_cast<T*>(block);
    for (size_t i = 0; i < dim; i++) {
      std::cout << ptr[i] << std::endl;
      break;
    }
    auto* num = reinterpret_cast<int32_t*>(block + sizeof(T) * dim);
    std::cout << "num: " << *num << std::endl;
    break;
  }
  return 0;
}

template <typename T>
static inline size_t gen_train_data(T* from, T* train_data, float p_val,
                                    size_t N, size_t dim) {
  auto train_idx = random_split(N, p_val);
  size_t iter = 0;
  for (auto& idx : train_idx) {
    std::memcpy(train_data + iter * dim, from + idx * dim, sizeof(T) * dim);
    iter++;
  }
  return train_idx.size();
}

template <typename T>
static inline void fetch_sub_data(T* from, T* sub_data, size_t id, size_t N,
                                  size_t dim, size_t sdim) {
  size_t offset = sdim * id;
  for (size_t i = 0; i < N; i++) {
    std::memcpy(sub_data + sdim * i, from + i * dim + offset, sizeof(T) * sdim);
  }
}

template <typename T>
void VamanaIndex<T>::gen_pq_index() {
  float p_val = TRAINING_SET_SIZE * 1.0 / option.N;

  std::unique_ptr<T[]> train_data =
      std::make_unique<T[]>(option.N * option.dim);

  std::unique_ptr<T[]> sub_data = std::make_unique<T[]>(option.N * option.sdim);

  std::vector<std::shared_ptr<T[]>> centroid_list;

  size_t size =
      gen_train_data(_data, train_data.get(), p_val, option.N, option.dim);

  for (size_t id = 0; id < option.M; id++) {
    std::shared_ptr<T[]> cent_data(new T[option.ksub * option.sdim]);
    std::memset(sub_data.get(), 0, sizeof(T) * option.N * option.sdim);
    fetch_sub_data<T>(train_data.get(), sub_data.get(), id, size, option.dim,
                      option.sdim);

    kmeans_cluster(train_data.get(), size, option.sdim, cent_data.get(),
                   option.ksub, 12);

    centroid_list.emplace_back(cent_data);
  }

  std::vector<T*> centroid_ptrs;
  centroid_ptrs.reserve(centroid_list.size());
  for (auto& cent : centroid_list) {
    centroid_ptrs.emplace_back(cent.get());
  }

  // #pragma omp parallel for schedule(dynamic, 256)
  for (size_t i = 0; i < 6; i++) {
    auto codes = compute_pq_code(vec_ptr[i], centroid_ptrs, option.sdim);
    std::cout << fmt::format("code: [{}]\n", fmt::join(codes, ", "));
  }
}

template class VamanaIndex<float>;

}  // namespace vamana