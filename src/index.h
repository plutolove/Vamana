#pragma once
#include <assert.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <set>
#include <unordered_set>

#include "boost/noncopyable.hpp"
#include "index_option.h"

namespace vamana {
template <typename T>
struct Entry {
  T* ptr;
};

template <typename T, typename DistCalc>
class VamanaIndex : boost::noncopyable {
 public:
  VamanaIndex(const IndexOption<DistCalc>& option) : option(option) {
    loadData();
    std::cout << "start generate random graph\n";
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // 随机编号
    std::uniform_int_distribution<int> dist(0, option.N - 1);
    // 初始化随机图
    _graph.resize(option.N);
    for (auto& child : _graph) {
      child.reserve(option.R);
      size_t cnt = option.R;
      while (cnt--) {
        child.emplace_back(dist(generator));
      }
    }
    std::cout << fmt::format("generate random graph finished, R: {}\n",
                             option.R);
  }

  size_t calcCentroid() {
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
    return idx;
  }
  // 起点，查询点，top k， search list size， topk res，vis list
  size_t bfs_search(size_t s, const T* q, size_t k, size_t L) {
    std::cout << "bfs_search -------" << std::endl;
    // 最大距离，用于限制进队列的数据量，减少搜索空间
    // 在搜索结果小于k时默认为最大值，否则为当前搜索的k个点的距离的最大值
    T max_dist = std::numeric_limits<T>::max();

    // topk的dist和idx
    std::set<std::pair<T, size_t>> topk;

    // 遍历过的点
    std::unordered_set<size_t> visit{s};

    // 小根堆保存dist和idx
    using PI = std::pair<T, size_t>;
    std::priority_queue<PI, std::deque<PI>, std::greater<PI>> query;
    // std::priority_queue<> query;
    query.push(std::make_pair(option.calc(vec_ptr[s], q, option.dim), s));
    // bfs搜索
    while (not query.empty()) {
      auto front = query.top();
      std::cout << fmt::format("------ visit idx: {}\n", front.second);
      query.pop();
      auto& childred = _graph[front.second];
      for (auto child : childred) {
        T cur_dist = option.calc(q, vec_ptr[child], option.dim);
        // 之前没有访问的点，同时距离小于最大距离
        if (0 == visit.count(child) && cur_dist <= max_dist) {
          PI kv = std::make_pair(cur_dist, child);
          // 入队列
          query.push(kv);
          // 标记遍历过了
          visit.insert(child);
          // 保存到topk结果中
          topk.insert(kv);
        }
      }
      // 如果topk的size大于L，则删除多余的距离大的点
      if (topk.size() > L) {
        max_dist = std::min(topk.rbegin()->first, max_dist);
        std::cout << fmt::format("topk size: {}, L: {}\n", topk.size(), L);
        size_t del_cnt = topk.size() - L;
        for (auto iter = topk.rbegin(); iter != topk.rend(); iter++) {
          del_cnt--;
          topk.erase(*iter);
          if (0 == del_cnt) break;
        }
      }
    }
    std::cout << fmt::format("visit node size: {}\n", visit.size());
    return topk.begin()->second;
  }

  void build() {
    std::vector<size_t> index_data(option.N);
    std::iota(index_data.begin(), index_data.end(), 0);
    // 随机打散所有点，用于随机遍历所有点，构建索引
    std::random_shuffle(index_data.begin(), index_data.end());
    size_t ep_idx = calcCentroid();
    auto ridx = bfs_search(ep_idx, vec_ptr[0], 1, 20);
    std::cout << fmt::format(
        "search res: {}, except idx: {}, dist of q and res: {}\n", ridx, 0,
        option.calc(vec_ptr[0], vec_ptr[ridx], option.dim));
  }

  ~VamanaIndex() {
    // 释放内存
    delete[] _data;
  }

 protected:
  void loadData() {
    std::cout << fmt::format("start load data from: {}\n", option.file_name);
    std::ifstream in(option.file_name, std::ios_base::binary);
    size_t dim;
    size_t N;
    in.read(reinterpret_cast<char*>(&N), sizeof(size_t));
    in.read(reinterpret_cast<char*>(&dim), sizeof(dim));
    assert(option.N == N);
    assert(option.dim == dim);
    vec_ptr.reserve(N);
    _data = new T[N * dim];
    T* ptr = _data;
    for (size_t i = 0; i < N; ++i) {
      in.read(reinterpret_cast<char*>(ptr), dim * sizeof(T));
      vec_ptr.push_back(ptr);
      ptr += dim;
    }
    std::cout << fmt::format("read data finished, size: {}, dim: {}\n", N, dim);
    in.close();
  }

 protected:
  // 所有vec保存到同一个_data数组中
  T* _data;
  // 保存每个vec的起始ptr
  std::vector<const T*> vec_ptr;
  std::vector<std::vector<size_t>> _graph;
  IndexOption<DistCalc> option;
};

}  // namespace vamana