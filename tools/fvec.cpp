#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

#include "fmt/format.h"

using namespace std;

int main() {
  ifstream fin("../data/siftsmall_learn.fvecs", std::ios_base::binary);
  ofstream out("../data/data.bin", std::ios_base::binary);
  int idim;
  fin.read((char*)&idim, 4);  //读取向量维度
  size_t dim = idim;
  fin.seekg(0, std::ios::end);          //光标定位到文件末尾
  std::ios::pos_type ss = fin.tellg();  //获取文件大小（多少字节）
  size_t fsize = (size_t)ss;
  size_t N = (unsigned)(fsize / (dim + 1) / 4);  //数据的个数
  auto* data = new float[dim];

  std::cout << "N: " << N << " dim: " << dim << std::endl;

  out.write(reinterpret_cast<char*>(&N), sizeof(N));
  out.write(reinterpret_cast<char*>(&dim), sizeof(dim));
  fin.seekg(0, std::ios::beg);  //光标定位到起始处
  for (size_t i = 0; i < N; i++) {
    fin.seekg(4, std::ios::cur);  //光标向右移动4个字节
    fin.read(reinterpret_cast<char*>(data),
             dim * sizeof(float));  //读取数据到一维数据data中
    out.write(reinterpret_cast<char*>(data), dim * sizeof(float));
  }
  delete[] data;
  out.close();
  return 0;
}