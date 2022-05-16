#include <string.h>

#include <cassert>
#include <iostream>
#include <list>
#include <memory>
#include <queue>

#include "block.h"
#include "block_cache.h"
#include "block_pool.h"
#include "common/define.h"
#include "common/exception.h"
#include "disk_index.h"
#include "distance.h"
#include "fmt/format.h"
#include "gflags/gflags.h"
#include "index.h"
#include "index_option.h"

DEFINE_uint64(N, 80000, "data size");
DEFINE_uint64(dim, 100, "vector dim");
DEFINE_uint64(L, 35, "search L size");
DEFINE_uint64(R, 25, "graph degree");
DEFINE_string(data_path, "../data/data.bin", "input data file path");
DEFINE_string(index_path, "../data/index.bin", "index save path");
DEFINE_string(teat_data_path, "../data/test.bin", "test data path");
DEFINE_int32(thread_num, 16, "openmp thread num");
using namespace vamana;

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
  VamanaIndex<float> index(option);
  // index.build();
  // index.save_disk_index("../data/disk_index.bin");
  //  index.load_disk_index("../data/disk_index.bin");
  //  index.save_index();
  //  index.load_index();
  //  index.test();

  DiskIndex<float> dindex("../data/disk_index.bin");
  BlockCache<int32_t, BlockPtr> cache(1024);
  auto block_ptr = BlockPool::getInstance().getSingleBlockPtr();
  SharedBlockCache scache(8, 1024);
  scache.insert(0, block_ptr);
  auto ret = scache.find(0);
  if (ret)
    assert(int64_t(ret->value) == int64_t(block_ptr));
  else
    std::cout << "ret is nullptr" << std::endl;
  scache.erase(0);

  return 0;
}
