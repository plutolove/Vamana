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
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "index_option.h"

namespace vamana {
template <typename T>
struct Entry {
  T* ptr;
};

template <typename T, typename DistCalc>
class VamanaIndex {
  // <dist, idx>
  using PI = std::pair<T, size_t>;

 public:
  VamanaIndex(const IndexOption<DistCalc>& option) : option(option) {
    loadData();
    _graph.resize(option.N);
  }

  void init_random_graph() {
    // 初始化随机图
    std::cout << "start generate random graph\n";
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // 随机编号
    std::uniform_int_distribution<int> dist(0, option.N - 1);
    for (size_t i = 0; i < _graph.size(); i++) {
      auto& child = _graph[i];
      std::unordered_set<size_t> gen_child;
      while (child.size() < option.R) {
        // 随机图中的child节点不重复，同时节点编号不是自己
        size_t idx = dist(generator);
        if (idx == i) continue;
        child.insert(idx);
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
    option.centroid_idx = idx;
    return idx;
  }

  // 起点，查询点，top k， search list size， topk res，vis list
  size_t bfsSearch(size_t s, const T* q, size_t k, size_t L,
                   std::set<std::pair<T, size_t>>& topk,
                   std::set<std::pair<T, size_t>>& V) {
    // 最大距离，用于限制进队列的数据量，减少搜索空间
    // 在搜索结果小于k时默认为最大值，否则为当前搜索的k个点的距离的最大值
    T max_dist = std::numeric_limits<T>::max();

    // 遍历过的点
    std::unordered_set<size_t> visit;

    // 小根堆保存dist和idx
    std::priority_queue<PI, std::deque<PI>, std::greater<PI>> query;
    // std::priority_queue<> query;
    query.push(std::make_pair(option.calc(vec_ptr[s], q, option.dim), s));
    // bfs搜索
    while (not query.empty()) {
      auto front = query.top();
      query.pop();
      // 保存到topk结果中
      topk.insert(front);
      // 标记遍历过了
      visit.insert(front.second);
      // 直接返回访问过的点和距离，后续可以直接使用，不用重复计算
      V.insert(front);

      auto& childred = _graph[front.second];
      for (auto child : childred) {
        T cur_dist = option.calc(q, vec_ptr[child], option.dim);
        // 之前没有访问的点，同时距离小于最大距离
        if (0 == visit.count(child) && cur_dist <= max_dist) {
          PI kv = std::make_pair(cur_dist, child);
          // 入队列
          query.push(kv);
        }
      }
      // 如果topk的size大于L，则删除多余的距离大的点
      if (topk.size() > L) {
        max_dist = std::min(topk.rbegin()->first, max_dist);
        size_t del_cnt = topk.size() - L;
        for (auto iter = topk.rbegin(); iter != topk.rend(); iter++) {
          del_cnt--;
          topk.erase(*iter);
          if (0 == del_cnt) break;
        }
      }
    }

    // std::cout << fmt::format("visit node size: {}\n", visit.size());
    // 返回最近点的idx
    return topk.begin()->second;
  }

  size_t robustPrune(size_t idx, std::set<std::pair<T, size_t>>& V, float e,
                     size_t R) {
    // 构建候选点，删除自己
    std::set<std::pair<T, size_t>> candidate;
    for (auto& kv : V) {
      if (kv.second == idx) continue;
      candidate.insert(kv);
    }

    std::unordered_set<size_t> Nout;
    while (not candidate.empty()) {
      auto val = *candidate.begin();
      Nout.insert(val.second);
      if (Nout.size() == option.R) break;

      std::set<std::pair<T, size_t>> new_candi;
      for (auto& kv : candidate) {
        auto dist = e * option.calc(vec_ptr[val.second], vec_ptr[kv.second],
                                    option.dim);
        if (dist <= option.calc(vec_ptr[idx], vec_ptr[kv.second], option.dim)) {
          continue;
        }
        new_candi.insert(kv);
      }
      std::swap(candidate, new_candi);
    }
    std::swap(_graph[idx], Nout);
    return _graph[idx].size();
  }

  size_t robustPruneAll(size_t from, float e) {
    auto& children = _graph[from];
    // from -> to -> ed;
    for (auto& idx : children) {
      auto& ed = _graph[idx];
      if (ed.size() < option.R) {
        ed.insert(from);
      } else {
        std::set<std::pair<T, size_t>> V{
            {option.calc(vec_ptr[idx], vec_ptr[from], option.dim), from}};
        for (auto& j : ed) {
          V.insert(std::make_pair(
              option.calc(vec_ptr[idx], vec_ptr[j], option.dim), j));
        }
        robustPrune(idx, V, e, option.R);
      }
    }
    return 0;
  }

  void build() {
    init_random_graph();
    std::vector<size_t> index_data(option.N);
    std::iota(index_data.begin(), index_data.end(), 0);
    // 随机打散所有点，用于随机遍历所有点，构建索引
    std::random_shuffle(index_data.begin(), index_data.end());
    size_t ep_idx = calcCentroid();
    size_t cnt = 0;
    for (auto& idx : index_data) {
      std::set<std::pair<T, size_t>> topk;
      std::set<std::pair<T, size_t>> visit;
      auto ridx = bfsSearch(ep_idx, vec_ptr[idx], 1, option.L, topk, visit);
      // std::cout << fmt::format(
      //     "search res: {}, except idx: {}, dist of q and res: {}\n", ridx,
      //     idx, option.calc(vec_ptr[idx], vec_ptr[ridx], option.dim));
      robustPrune(idx, visit, 1, option.R);
      robustPruneAll(idx, 1);
      // 随机测试5
    }
    std::cout << "-----------" << std::endl;
    for (auto& idx : index_data) {
      std::set<std::pair<T, size_t>> topk;
      std::set<std::pair<T, size_t>> visit;
      auto ridx = bfsSearch(ep_idx, vec_ptr[idx], 1, option.L, topk, visit);
      // std::cout << fmt::format(
      //     "search res: {}, except idx: {}, dist of q and res: {}\n", ridx,
      //     idx, option.calc(vec_ptr[idx], vec_ptr[ridx], option.dim));
      robustPrune(idx, visit, 1.2, option.R);
      robustPruneAll(idx, 1.2);
    }
  }

  size_t read_index() {
    std::cout << "start read index" << std::endl;
    std::ifstream fin(option.save_path, std::ios::binary);
    size_t N;
    size_t ves[500];
    fin.read(reinterpret_cast<char*>(&N), sizeof(N));
    std::cout << "read N: " << N << std::endl;
    assert(N == option.N);
    for (size_t i = 0; i < option.N; ++i) {
      size_t size;
      size_t from;
      // 起点
      fin.read(reinterpret_cast<char*>(&from), sizeof(size_t));
      // 链接的size
      fin.read(reinterpret_cast<char*>(&size), sizeof(size_t));
      fin.read(reinterpret_cast<char*>(ves), sizeof(size_t) * size);
      _graph[from].clear();
      // 所有相连的点
      for (size_t j = 0; j < size; j++) {
        _graph[from].insert(ves[j]);
      }
    }
    fin.close();
    return option.N;
  }

  size_t save_index() {
    std::cout << "start save index" << std::endl;
    std::ofstream fout(option.save_path, std::ios::binary);
    size_t N = option.N;
    fout.write(reinterpret_cast<char*>(&N), sizeof(N));
    std::cout << "write N: " << N << std::endl;
    for (size_t i = 0; i < option.N; ++i) {
      const auto& edgs = _graph[i];
      size_t size = edgs.size();
      // 起点
      fout.write(reinterpret_cast<char*>(&i), sizeof(size_t));
      // 链接的size
      fout.write(reinterpret_cast<char*>(&size), sizeof(size_t));
      size_t ves[500];
      size_t idx = 0;
      // 所有相连的点
      for (auto& v : edgs) {
        ves[idx++] = v;
      }
      fout.write(reinterpret_cast<const char*>(ves),
                 sizeof(size_t) * edgs.size());
    }
    fout.close();
    return option.N;
  }

  size_t forceSearch(const T* ptr) {
    T dist = option.calc(ptr, vec_ptr[0], option.dim);
    size_t ret = 0;
    for (size_t i = 0; i < option.N; i++) {
      auto d = option.calc(ptr, vec_ptr[i], option.dim);
      if (d < dist) {
        dist = d;
        ret = i;
      }
    }
    return ret;
  }

  size_t test() {
    loadTest();
    size_t same = 0;
    size_t diff = 0;
    for (size_t i = 0; i < option.test_N; i++) {
      std::set<std::pair<T, size_t>> topk;
      std::set<std::pair<T, size_t>> visit;
      auto ridx = bfsSearch(option.centroid_idx, _test_ptr[i], 1, option.L,
                            topk, visit);
      auto fidx = forceSearch(_test_ptr[i]);
      if (fidx == ridx)
        same++;
      else
        diff++;
      std::cout << fmt::format(
          "search res: {}, except idx: {}, force dist: {}, ann dist: {}\n",
          ridx, fidx, option.calc(vec_ptr[fidx], _test_ptr[i], option.dim),
          option.calc(vec_ptr[ridx], _test_ptr[i], option.dim));
    }
    std::cout << fmt::format("test same: {}, diff: {}, same rate: {}\n", same,
                             diff, same * 1.0 / (same + diff));
    return 0;
  }

  ~VamanaIndex() {
    // 释放内存
    if (_data) delete[] _data;
    if (_test) delete[] _test;
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

  void loadTest() {
    std::cout << fmt::format("start load data from: {}\n", option.test_file);
    std::ifstream in(option.test_file, std::ios_base::binary);
    size_t dim;
    size_t N;
    in.read(reinterpret_cast<char*>(&N), sizeof(size_t));
    in.read(reinterpret_cast<char*>(&dim), sizeof(dim));
    assert(option.test_N == N);
    assert(option.dim == dim);
    _test_ptr.reserve(N);
    _test = new T[N * dim];
    T* ptr = _test;
    for (size_t i = 0; i < N; ++i) {
      in.read(reinterpret_cast<char*>(ptr), dim * sizeof(T));
      _test_ptr.push_back(ptr);
      ptr += dim;
    }
    std::cout << fmt::format("read data finished, size: {}, dim: {}\n", N, dim);
    in.close();
  }

 protected:
  // 所有vec保存到同一个_data数组中
  T* _data = nullptr;
  T* _test = nullptr;
  std::vector<const T*> _test_ptr;
  // 保存每个vec的起始ptr
  std::vector<const T*> vec_ptr;
  std::vector<std::unordered_set<size_t>> _graph;
  IndexOption<DistCalc> option;
};

}  // namespace vamana