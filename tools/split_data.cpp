#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

#include "fmt/format.h"

using namespace std;

int main() {
  ifstream fin("../data/query100K.fbin", std::ios_base::binary);
  ofstream out("../data/data.bin", std::ios_base::binary);
  ofstream tout("../data/test.bin", std::ios_base::binary);
  int iN, idim;
  fin.read((char*)&iN, 4);  //读取向量维度
  fin.read((char*)&idim, 4);
  size_t dim = idim;
  size_t N = iN;

  // 读取所有数据
  float* data = new float[dim * N];
  std::vector<float*> vec_ptr;
  auto* ptr = data;
  for (size_t i = 0; i < N; i++) {
    fin.read(reinterpret_cast<char*>(ptr), dim * sizeof(float));
    vec_ptr.push_back(ptr);
    ptr += dim;
  }

  // 随机shuffle
  std::random_shuffle(vec_ptr.begin(), vec_ptr.end());

  N = 80000;
  out.write(reinterpret_cast<char*>(&N), sizeof(N));
  out.write(reinterpret_cast<char*>(&dim), sizeof(dim));
  for (size_t i = 0; i < N; i++) {
    out.write(reinterpret_cast<char*>(vec_ptr[i]), dim * sizeof(float));
  }
  out.close();

  N = 20000;
  tout.write(reinterpret_cast<char*>(&N), sizeof(N));
  tout.write(reinterpret_cast<char*>(&dim), sizeof(dim));
  for (size_t i = 80000; i < 100000; i++) {
    tout.write(reinterpret_cast<char*>(vec_ptr[i]), dim * sizeof(float));
  }
  delete[] data;
  fin.close();
  tout.close();
  return 0;
}