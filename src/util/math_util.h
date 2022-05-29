#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <random>
#include <unordered_set>

#include "fmt/format.h"
#include "index/distance.h"

namespace vamana {

static inline std::vector<size_t> random_split(size_t N, float p_val) {
  if (p_val > 1) p_val = 1;
  std::random_device rd;
  size_t x = rd();
  std::mt19937 generator((unsigned)x);
  std::uniform_real_distribution<float> distribution(0, 1);
  std::vector<size_t> ret;
  for (size_t i = 0; i < N; i++) {
    float p = distribution(generator);
    if (p < p_val) {
      ret.emplace_back(i);
    }
  }
  return ret;
}

template <typename T>
static void kmeanspp_init(T* train_data, size_t num_points, size_t dim,
                          T* cent_data, size_t num_centers) {
  std::unordered_set<size_t> picked;
  std::random_device rd;
  auto x = rd();
  std::mt19937 generator(x);
  std::uniform_real_distribution<> distribution(0, 1);
  std::uniform_int_distribution<size_t> int_dist(0, num_points - 1);
  size_t init_id = int_dist(generator);
  size_t num_picked = 1;

  static DistanceL2<T> calc;

  picked.insert(init_id);
  std::memcpy(cent_data, train_data + init_id * dim, dim * sizeof(T));

  T* dist = new T[num_points];

#pragma omp parallel for schedule(static, 8192)
  for (size_t i = 0; i < num_points; i++) {
    dist[i] = calc(train_data + i * dim, train_data + init_id * dim, dim);
  }

  double dart_val;
  size_t tmp_pivot;
  bool sum_flag = false;

  while (num_picked < num_centers) {
    dart_val = distribution(generator);

    double sum = 0;
    for (size_t i = 0; i < num_points; i++) {
      sum = sum + dist[i];
    }
    if (sum == 0) sum_flag = true;

    dart_val *= sum;

    double prefix_sum = 0;
    for (size_t i = 0; i < num_points; i++) {
      tmp_pivot = i;
      if (dart_val >= prefix_sum && dart_val < prefix_sum + dist[i]) {
        break;
      }

      prefix_sum += dist[i];
    }

    if (picked.find(tmp_pivot) != picked.end() && (sum_flag == false)) continue;
    picked.insert(tmp_pivot);
    std::memcpy(cent_data + num_picked * dim, train_data + tmp_pivot * dim,
                dim * sizeof(T));
#pragma omp parallel for schedule(static, 8192)
    for (size_t i = 0; i < num_points; i++) {
      dist[i] = (std::min)(dist[i], calc(train_data + i * dim,
                                         train_data + tmp_pivot * dim, dim));
    }
    num_picked++;
  }
  delete[] dist;
}

template <typename T>
static T compute_closest_centers(T* train_data, size_t num_points, size_t dim,
                                 T* cent_data, size_t num_centers,
                                 std::vector<size_t>& cent_id) {
  static DistanceL2<T> calc;
  std::vector<T> dis_list;
  dis_list.resize(num_points, 0);

#pragma omp parallel for schedule(dynamic, 256)
  for (size_t i = 0; i < num_points; i++) {
    size_t id = 0;
    T dis = calc(train_data + i * dim, cent_data + 0 * dim, dim);
    for (size_t j = 0; j < num_centers; j++) {
      auto tmp_dis = calc(train_data + i * dim, cent_data + j * dim, dim);
      if (tmp_dis < dis) {
        dis = tmp_dis;
        id = j;
      }
    }
    cent_id[i] = id;
    dis_list[i] = dis;
  }
  T sum = 0;
  for (auto& v : dis_list) sum += v;
  return sum / num_points;
}

template <typename T>
static T kmeans_iter(T* train_data, size_t num_points, size_t dim, T* cent_data,
                     size_t num_centers, std::vector<size_t>& cent_id) {
  std::vector<std::vector<size_t>> cent_map(num_centers);

  //归类cent
  for (size_t i = 0; i < cent_id.size(); i++)
    cent_map[cent_id[i]].emplace_back(i);

#pragma omp parallel for schedule(dynamic, 1)
  for (size_t i = 0; i < num_centers; i++) {
    T tmp[1000];
    std::memset(tmp, 0, sizeof(tmp));

    for (auto& id : cent_map[i]) {
      auto* ptr = train_data + id * dim;
      for (size_t d = 0; d < dim; d++) {
        tmp[d] += ptr[d];
      }
    }

    for (size_t d = 0; d < dim; d++) {
      tmp[d] /= cent_map[i].size();
    }
    // 更新cent
    std::memcpy(cent_data + i * dim, tmp, sizeof(T) * dim);
  }

  return compute_closest_centers(train_data, num_points, dim, cent_data,
                                 num_centers, cent_id);
}

template <typename T>
static void kmeans_cluster(T* train_data, size_t num_points, size_t dim,
                           T* cent_data, size_t num_centers, size_t iter_num) {
  std::cout << fmt::format(
      "kmeans data size: {}, cent size: {}, dim: {}, iter_num: {}\n",
      num_points, num_centers, dim, iter_num);
  kmeanspp_init(train_data, num_points, dim, cent_data, num_centers);

  std::vector<size_t> cent_id(num_points, 0);

  // 计算最近的cent
  T residual = compute_closest_centers(train_data, num_points, dim, cent_data,
                                       num_centers, cent_id);
  size_t iter = 0;

  while (iter < iter_num) {
    T old_residual = residual;

    residual = kmeans_iter(train_data, num_points, dim, cent_data, num_centers,
                           cent_id);

    if (iter > 0 && (residual < std::numeric_limits<T>::epsilon() ||
                     ((old_residual - residual) / residual) < 0.00001))
      break;
    std::cout << fmt::format("iter {} residual: {}\n", iter + 1, residual);
    iter++;
  }
}

template <typename T>
static inline uint8_t compute_pq_code(const T* query, const T* cent_ptr,
                                      size_t dim, size_t cluster_num = 256) {
  static DistanceL2<T> calc;
  uint8_t ret = 0;
  T dist = std::numeric_limits<T>::max();
  for (size_t i = 0; i < cluster_num; i++) {
    auto tmp_dist = calc(query, cent_ptr + i * dim, dim);
    if (tmp_dist < dist) {
      dist = tmp_dist;
      ret = i;
    }
  }
  return ret;
}

template <typename T>
static inline std::vector<uint8_t> compute_pq_code(const T* query,
                                                   std::vector<T*> cent_ptrs,
                                                   size_t dim,
                                                   size_t cluster_num = 256) {
  static DistanceL2<T> calc;
  std::vector<uint8_t> ret;
  ret.reserve(cent_ptrs.size());
  for (size_t cent_id = 0; cent_id < cent_ptrs.size(); cent_id++) {
    auto* cent_ptr = cent_ptrs[cent_id];
    auto* query_sub = query + cent_id * dim;
    size_t code = 0;
    T dist = std::numeric_limits<T>::max();
    for (size_t i = 0; i < cluster_num; i++) {
      auto tmp_dist = calc(query_sub, cent_ptr + i * dim, dim);
      if (tmp_dist < dist) {
        dist = tmp_dist;
        code = i;
      }
    }
    ret.emplace_back(code);
  }
  return ret;
}

template <typename T>
static inline std::vector<uint8_t> compute_pq_code(
    const T* query, std::vector<std::shared_ptr<T[]>> cent_ptrs, size_t dim,
    size_t cluster_num = 256) {
  static DistanceL2<T> calc;
  std::vector<uint8_t> ret;
  ret.reserve(cent_ptrs.size());
  for (size_t cent_id = 0; cent_id < cent_ptrs.size(); cent_id++) {
    auto* cent_ptr = cent_ptrs[cent_id].get();
    auto* query_sub = query + cent_id * dim;
    size_t code = 0;
    T dist = std::numeric_limits<T>::max();
    for (size_t i = 0; i < cluster_num; i++) {
      auto tmp_dist = calc(query_sub, cent_ptr + i * dim, dim);
      if (tmp_dist < dist) {
        dist = tmp_dist;
        code = i;
      }
    }
    ret.emplace_back(code);
  }
  return ret;
}

}  // namespace vamana