#include <random>

#include "block_cache.h"
#include "gtest/gtest.h"
using namespace vamana;
TEST(TestCache, multhead) {
  auto& pool = BlockPool::getInstance();
  BlockCache<int32_t, BlockPtr, BlockPool> cache(8);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  // 随机编号
  std::uniform_int_distribution<int> dist(0, 8);

  auto test = [&cache, &dist, &generator, &pool]() {
    for (size_t i = 0; i < 50; i++) {
      auto op = dist(generator) % 2;
      auto key = dist(generator);
      if (op == 0) {
        cache.insert(key, pool.getSingleBlockPtr(), 0);
      } else {
        auto handle = cache.find(key, 0);
        if (handle) {
          cache.Release(handle);
          std::cout << fmt::format("cached key: {}\n", handle->key);
        }
      }
    }
  };
  std::vector<std::thread> ts;
  for (size_t i = 0; i < 10; i++) {
    std::thread t(test);
    ts.push_back(std::move(t));
  }
  for (auto& t : ts) t.join();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  RUN_ALL_TESTS();
  return 0;
}