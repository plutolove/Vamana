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
  option.N = 80000;
  option.dim = 100;
  option.calc = dis;
  option.file_name = "../data/data.bin";
  option.L = 35;
  option.R = 25;
  option.save_path = "../data/index.bin";
  option.test_file = "../data/test.bin";
  option.test_N = 20000;
  VamanaIndex<float, DistanceL2Float> index(option);
  index.build();
  index.save_index();
  index.calcCentroid();
  index.load_index();
  index.test();
  return 0;
}