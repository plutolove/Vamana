#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
using namespace std;

int main() {
  ofstream out("data.bin", std::ios_base::binary);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  // 随机编号
  std::uniform_real_distribution<float> dist(0, 1);
  size_t dim = 100;
  size_t N = 10000;
  out.write(reinterpret_cast<char*>(&N), sizeof(N));
  out.write(reinterpret_cast<char*>(&dim), sizeof(dim));
  float vec[100];
  while (N--) {
    for (size_t i = 0; i < dim; i++) {
      vec[i] = dist(generator);
    }
    out.write(reinterpret_cast<char*>(vec), 100 * sizeof(float));
  }
  out.close();
  return 0;
}