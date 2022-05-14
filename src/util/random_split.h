#pragma once
#include <cstdlib>
#include <random>
#include <vector>

static inline std::vector<size_t> random_split(float p_val, size_t N) {
  std::vector<size_t> ret;
  if (p_val > 1) p_val = 1;
  ret.reserve(size_t(N * p_val * 1.2));
  std::random_device rd;
  size_t x = rd();
  std::mt19937 generator((unsigned)x);
  std::uniform_real_distribution<float> distribution(0, 1);
  for (size_t i = 0; i < N; i++) {
    float rnd_val = distribution(generator);
    if (rnd_val < p_val) ret.push_back(i);
  }
  return ret;
}