#include <iostream>

#include "common/exception.h"
#include "distance.h"
#include "fmt/format.h"
#include "index.h"
#include "index_option.h"

using namespace vamana;
int main() {
  DistanceL2Float dis;
  IndexOption<DistanceL2Float> option;
  option.N = 25000;
  option.dim = 128;
  option.calc = dis;
  option.file_name = "../data/data.bin";
  option.L = 20;
  option.R = 15;
  VamanaIndex<float, DistanceL2Float> index(option);
  index.build();
  return 0;
}