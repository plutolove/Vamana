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
  int iN, idim;
  fin.read((char*)&iN, 4);  //读取向量维度
  fin.read((char*)&idim, 4);
  size_t dim = idim;
  size_t N = iN > 10000 ? 10000 : iN;
  auto* data = new float[dim];

  std::cout << "N: " << N << " dim: " << dim << std::endl;

  out.write(reinterpret_cast<char*>(&N), sizeof(N));
  out.write(reinterpret_cast<char*>(&dim), sizeof(dim));
  for (size_t i = 0; i < N; i++) {
    fin.read(reinterpret_cast<char*>(data),
             dim * sizeof(float));  //读取数据到一维数据data中
    out.write(reinterpret_cast<char*>(data), dim * sizeof(float));
  }
  delete[] data;
  out.close();
  return 0;
}