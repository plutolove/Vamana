#pragma once
#include <boost/core/noncopyable.hpp>
#include <list>
#include <mutex>

#include "block.h"
#include "boost/lockfree/queue.hpp"

namespace vamana {
// 由于block的内存是512对齐的内存, 4K per block
// 避免频繁申请释放内存
class BlockPool : boost::noncopyable {
 public:
  static BlockPool& getInstance() {
    static BlockPool instance(1024 * 8);
    return instance;
  }

  // 初始化size大小的pool
  BlockPool(size_t size) : pool(size) {
    while (size--) {
      block_list.emplace_back();
      pool.push(&block_list.back());
    }
  }

  ~BlockPool() { block_list.clear(); }

  inline void recycle(BlockPtr ptr) { pool.push(ptr); }

  inline BlockPtr getSingleBlockPtr() {
    BlockPtr ret = nullptr;
    // 如果pool里面没有可用的block，则新建一个
    if (not pool.pop(ret)) {
      std::lock_guard lock(mtx);
      block_list.emplace_back();
      ret = &block_list.back();
    }
    return ret;
  }

  inline std::vector<BlockPtr> getBlockPtrs(size_t size) {
    std::vector<BlockPtr> ret;
    ret.reserve(size);
    while (size--) {
      ret.push_back(getSingleBlockPtr());
    }
    return ret;
  }

 protected:
  // block 内存管理
  std::list<Block> block_list;
  boost::lockfree::queue<BlockPtr> pool;
  mutable std::mutex mtx;
};

}  // namespace vamana