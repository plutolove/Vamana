#pragma once
#include "block.h"
#include "block_pool.h"
#include "folly/concurrency/ConcurrentHashMap.h"

namespace vamana {

class BlockCache {
 public:
  BlockCache(size_t cap) : capacity(cap), cache_(cap) {}

 protected:
  folly::ConcurrentHashMap<int32_t, BlockPtr> cache_;
  size_t capacity;
};

}  // namespace vamana