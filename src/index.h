#pragma once
#include <assert.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <random>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "index_option.h"

#define ROUND_UP(X, Y) \
  ((((uint64_t)(X) / (Y)) + ((uint64_t)(X) % (Y) != 0)) * (Y))

#define DIV_ROUND_UP(X, Y) (((uint64_t)(X) / (Y)) + ((uint64_t)(X) % (Y) != 0))

// round down X to the nearest multiple of Y
#define ROUND_DOWN(X, Y) (((uint64_t)(X) / (Y)) * (Y))

namespace vamana {

template <typename T, typename DistCalc>
class VamanaIndex {
  // <dist, idx>
  using PI = std::pair<T, size_t>;

 public:
  VamanaIndex(const IndexOption<DistCalc>& option) : option(option) {
    loadData();
    _graph.resize(option.N);
    _locks = std::vector<std::mutex>(option.N);
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
      child.reserve(std::ceil(option.R * 1.3 * 1.05));
      std::unordered_set<size_t> gen_child;
      while (gen_child.size() < option.R) {
        // 随机图中的child节点不重复，同时节点编号不是自己
        size_t idx = dist(generator);
        if (idx == i) continue;
        if (gen_child.count(idx)) continue;
        gen_child.insert(idx);
        child.push_back(idx);
      }
    }
    std::cout << fmt::format("generate random graph finished, R: {}\n",
                             option.R);
  }

  size_t calcCentroid();

  // 起点，查询点，top k， search list size， topk res，vis list
  size_t bfsSearch(size_t s, const T* q, size_t k, size_t L,
                   std::vector<std::pair<T, size_t>>& visited_nodes,
                   std::vector<std::pair<T, size_t>>& visited_idx);

  void build();

  size_t load_index() {
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
      _graph[from].reserve(option.R);
      // 所有相连的点
      for (size_t j = 0; j < size; j++) {
        _graph[from].push_back(ves[j]);
      }
    }
    fin.close();
    return option.N;
  }

  void occlude_list(std::vector<PI>& pool, float alpha, size_t degree,
                    size_t maxc, std::vector<PI>& result);

  void occlude_list(std::vector<PI>& pool, float alpha, size_t degree,
                    size_t maxc, std::vector<PI>& result,
                    std::vector<float>& occlude_factor);

  void prune_neighbors(size_t node_idx, size_t R, size_t C, float alpha,
                       std::vector<PI>& node_list,
                       std::vector<size_t>& pruned_list);

  void batch_update(size_t node_idx, const std::vector<size_t>& pruned_list,
                    size_t R, std::vector<bool>& need_to_sync);

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
    for (size_t i = 0; i < 100; i++) {
      std::vector<std::pair<T, size_t>> topk;
      std::vector<std::pair<T, size_t>> visit;
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
  std::vector<std::vector<size_t>> _graph;
  std::vector<std::mutex> _locks;
  IndexOption<DistCalc> option;
};

}  // namespace vamana