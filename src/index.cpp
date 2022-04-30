#include "distance.h"
#include "index.h"
#include "omp.h"

namespace vamana {

template <typename T, typename DistCalc>
size_t VamanaIndex<T, DistCalc>::calcCentroid() {
  std::vector<T> ce(option.dim, 0);
  // 初始化质心
  for (size_t i = 0; i < option.N; ++i) {
    for (size_t j = 0; j < option.dim; ++j) {
      ce[j] += vec_ptr[i][j];
    }
  }
  for (size_t i = 0; i < option.dim; ++i) ce[i] /= option.N;

  auto* ce_ptr = ce.data();
  T dist = option.calc(ce_ptr, vec_ptr[0], option.dim);
  size_t idx = 0;
  for (size_t i = 1; i < option.N; ++i) {
    auto tmp_dist = option.calc(ce_ptr, vec_ptr[i], option.dim);
    if (tmp_dist < dist) {
      dist = tmp_dist;
      idx = i;
    }
  }
  std::cout << fmt::format("calcCentroid idx: {}, dist: {}\n", idx, dist);
  option.centroid_idx = idx;
  return idx;
}

template <typename T, typename DistCalc>
size_t VamanaIndex<T, DistCalc>::bfsSearch(size_t s, const T* q, size_t k,
                                           size_t L,
                                           std::set<std::pair<T, size_t>>& topL,
                                           std::set<std::pair<T, size_t>>& V) {
  // 最大距离，用于限制进队列的数据量，减少搜索空间
  // 在搜索结果小于k时默认为最大值，否则为当前搜索的k个点的距离的最大值
  T max_dist = std::numeric_limits<T>::max();

  // 遍历过的点
  std::unordered_set<size_t> visit{s};
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
  return topL.rbegin()->second;
}

template <typename T, typename DistCalc>
void VamanaIndex<T, DistCalc>::build() {
  // 分NUM_SYNCS次batch执行
  size_t NUM_SYNCS = DIV_ROUND_UP(option.N, (64 * 64));
  size_t round_size = DIV_ROUND_UP(option.N, NUM_SYNCS);  // size of each batch

  init_random_graph();
  std::vector<size_t> index_data(option.N);
  std::iota(index_data.begin(), index_data.end(), 0);
  // 随机打散所有点，用于随机遍历所有点，构建索引
  std::random_shuffle(index_data.begin(), index_data.end());
  size_t ep_idx = calcCentroid();
  size_t cnt = 0;
  size_t same = 0;
  size_t diff = 0;
  for (size_t sync_num = 0; sync_num < NUM_SYNCS; sync_num++) {
    size_t start_id = sync_num * round_size;
    size_t end_id = std::min(option.N, (sync_num + 1) * round_size);
#pragma omp parallel for schedule(dynamic)
    for (size_t idx = start_id; idx < end_id; ++idx) {
      std::set<std::pair<T, size_t>> topk;
      std::set<std::pair<T, size_t>> visit;
      auto ridx = bfsSearch(ep_idx, vec_ptr[idx], 1, option.L, topk, visit);
      std::cout << fmt::format(
          "bfs search running, thread num: {}, thread id: {}\n",
          omp_get_num_threads(), omp_get_thread_num());
    }
  }
}

template class VamanaIndex<float, DistanceL2Float>;

}  // namespace vamana