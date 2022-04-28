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
  option.L = 15;
  option.R = 10;
  option.save_path = "../data/index.bin";
  VamanaIndex<float, DistanceL2Float> index(option);
  index.build();
  index.save_index();
  index.calcCentroid();
  index.read_index();
  return 0;
}