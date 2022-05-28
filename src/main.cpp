#include <string.h>

#include <cassert>
#include <iostream>
#include <list>
#include <memory>
#include <queue>
#include <random>
#include <ratio>

#include "common/define.h"
#include "common/exception.h"
#include "fmt/format.h"
#include "gflags/gflags.h"
#include "index/block.h"
#include "index/block_cache.h"
#include "index/block_pool.h"
#include "index/disk_index.h"
#include "index/distance.h"
#include "index/index.h"
#include "index/index_option.h"

DEFINE_uint64(N, 80000, "data size");
DEFINE_uint64(dim, 100, "vector dim");
DEFINE_uint64(L, 35, "search L size");
DEFINE_uint64(R, 25, "graph degree");
DEFINE_string(data_path, "../data/data.bin", "input data file path");
DEFINE_string(index_path, "../data/index.bin", "index save path");
DEFINE_string(teat_data_path, "../data/test.bin", "test data path");
DEFINE_int32(thread_num, 16, "openmp thread num");
using namespace vamana;

const int TEST_SIZE = 10000;

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << FLAGS_N << std::endl;
  DistanceL2<float> dis;
  IndexOption<float> option;
  option.N = FLAGS_N;
  option.dim = FLAGS_dim;
  option.calc = dis;
  option.file_name = FLAGS_data_path;
  option.L = FLAGS_L;
  option.R = FLAGS_R;
  option.save_path = FLAGS_index_path;
  option.test_file = FLAGS_teat_data_path;
  option.thread_num = FLAGS_thread_num;
  option.test_N = 20000;
  option.M = 10;
  option.sdim = 10;
  VamanaIndex<float> index(option);
  index.gen_pq_index();
  // index.build();
  // index.save_disk_index("../data/disk_index.bin");
  // index.load_disk_index("../data/disk_index.bin");
  // index.save_index();
  // index.load_index();
  // index.test();
  //   std::vector<float*> _test_ptr;
  //   float* _test;
  //   std::cout << fmt::format("start load data from: {}\n", option.test_file);
  //   std::ifstream in(option.test_file, std::ios_base::binary);
  //   size_t dim;
  //   size_t N;
  //   in.read(reinterpret_cast<char*>(&N), sizeof(size_t));
  //   in.read(reinterpret_cast<char*>(&dim), sizeof(dim));
  //   assert(option.test_N == N);
  //   assert(option.dim == dim);
  //   _test_ptr.reserve(N);
  //   _test = new float[N * dim];
  //   float* ptr = _test;
  //   for (size_t i = 0; i < N; ++i) {
  //     in.read(reinterpret_cast<char*>(ptr), dim * sizeof(float));
  //     _test_ptr.push_back(ptr);
  //     ptr += dim;
  //   }
  //   std::cout << fmt::format("read data finished, size: {}, dim: {}\n", N,
  //   dim); in.close();

  //   DiskIndex<float> dindex("../data/disk_index.bin", 8, 128, 3);

  //   auto s = std::chrono::high_resolution_clock::now();
  //   double time_cost[TEST_SIZE];
  // #pragma omp parallel for schedule(dynamic, 1)
  //   for (size_t i = 0; i < TEST_SIZE; i++) {
  //     auto st = std::chrono::high_resolution_clock::now();
  //     auto ret = dindex.search(_test_ptr[i], 1, option.L + 5, 4);
  //     std::chrono::duration<double, std::milli> diff =
  //         std::chrono::high_resolution_clock::now() - st;
  //     time_cost[i] = diff.count();
  //   }
  //   std::chrono::duration<double> diff =
  //       std::chrono::high_resolution_clock::now() - s;
  //   double res = 0;
  //   for (size_t i = 0; i < TEST_SIZE; i++) {
  //     res += time_cost[i];
  //   }
  //   std::cout << fmt::format("search time cost avg: {}\n", res / TEST_SIZE);
  //   std::cout << fmt::format("search time cost: {}s\n", diff.count());
  //   delete[] _test;
  return 0;
}
