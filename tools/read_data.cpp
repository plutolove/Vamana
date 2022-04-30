#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
using namespace std;

int main() {
  ifstream in("../data/test.bin", std::ios_base::binary);
  size_t dim;
  size_t N;
  in.read(reinterpret_cast<char*>(&N), sizeof(size_t));
  in.read(reinterpret_cast<char*>(&dim), sizeof(dim));
  std::cout << "N: " << N << " dim: " << dim << std::endl;
  size_t top = 5;
  top = std::min(N, top);
  float vec[100];
  while (top--) {
    in.read(reinterpret_cast<char*>(vec), 100 * sizeof(float));
    for (size_t i = 0; i < dim; i++) {
      std::cout << vec[i] << ", ";
    }
    std::cout << endl;
  }
  in.close();
  return 0;
}