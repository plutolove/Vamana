#pragma once
#include <atomic>
#include <boost/core/noncopyable.hpp>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "block.h"
#include "block_pool.h"
#include "folly/concurrency/ConcurrentHashMap.h"

namespace vamana {

const uint32_t kInCacheBit = 1;
const uint32_t kUsageBit = 2;
const uint32_t kRefsOffset = 2;
const uint32_t kOneRef = 1 << kRefsOffset;

// clock 替换算法实现的cache
template <typename K, typename V, typename POOL>
class BlockCache : boost::noncopyable {
 public:
  struct CacheHandle {
    CacheHandle() = default;

    CacheHandle(const CacheHandle& a) { *this = a; }

    CacheHandle(const K& k, V& v) : key(k), value(v) {}

    CacheHandle& operator=(const CacheHandle& a) {
      key = a.key;
      value = a.value;
      return *this;
    }

    K key;
    V value;
    uint32_t hash;
    std::atomic<uint32_t> flags;
  };

  BlockCache(size_t cap) : capacity(cap), cache_(cap), head_(0) {}
  ~BlockCache() { list_.clear(); }

  static bool InCache(uint32_t flags) { return flags & kInCacheBit; }
  static bool HasUsage(uint32_t flags) { return flags & kUsageBit; }
  static uint32_t CountRefs(uint32_t flags) { return flags >> kRefsOffset; }

  inline void recycleHandle(CacheHandle* handle) {
    static auto& pool = POOL::getInstance();
    // 回收block
    pool.recycle(handle->value);
    handle->value = nullptr;
    handle->key = -1;
    // 回收handle
    recycle_.push_back(handle);
    size_.fetch_sub(1, std::memory_order_relaxed);
  }

  inline bool try_make_room(CacheHandle* handle) {
    uint32_t flags = kInCacheBit;
    // 如果flags == kInCacheBit，表示没有引用，则flags设置为0,回收handle
    if (handle->flags.compare_exchange_strong(
            flags, 0, std::memory_order_acquire, std::memory_order_relaxed)) {
      cache_.erase(handle->key);
      recycleHandle(handle);
      return true;
    }
    // 不能释放则usage 置0
    handle->flags.fetch_and(~kUsageBit, std::memory_order_relaxed);
    return false;
  }

  bool make_room() {
    auto size = size_.load(std::memory_order_relaxed);
    auto cap = capacity.load(std::memory_order_relaxed);

    size_t new_head = head_;
    bool second_iter = false;
    if (size >= cap) {
      while (size >= cap) {
        if (try_make_room(&list_[new_head])) {
          size = size_.load(std::memory_order_relaxed);
        }
        new_head = (new_head + 1 >= list_.size()) ? 0 : new_head + 1;
        if (new_head == head_) {
          // 第二次迭代还是没有可以释放的handle直接返回
          if (second_iter) {
            return false;
          } else {
            second_iter = true;
          }
        }
      }
    }
    head_ = new_head;
    return true;
  }

  inline bool UnsetInCache(CacheHandle* handle) {
    bool ret = false;
    uint32_t flags =
        handle->flags.fetch_and(~kInCacheBit, std::memory_order_acq_rel);
    // 如果之前的flags是incache，同时ref cnt=0则可以回收handle
    // 否则说明还有引用
    if (InCache(flags) && CountRefs(flags) == 0) {
      recycleHandle(handle);
      ret = true;
    }
    return ret;
  }

  inline bool Ref(CacheHandle* h) {
    auto handle = reinterpret_cast<CacheHandle*>(h);
    // CAS loop to increase reference count.
    uint32_t flags = handle->flags.load(std::memory_order_relaxed);
    while (InCache(flags)) {
      if (handle->flags.compare_exchange_weak(flags, flags + kOneRef,
                                              std::memory_order_acquire,
                                              std::memory_order_relaxed)) {
        return true;
      }
    }
    return false;
  }

  inline bool Unref(CacheHandle* handle) {
    bool ret = false;
    uint32_t flags =
        handle->flags.fetch_sub(kOneRef, std::memory_order_acq_rel);
    assert(CountRefs(flags) > 0);
    if (CountRefs(flags) == 1) {
      if (!InCache(flags)) {
        // 回收handle
        std::lock_guard lock(mtx);
        recycleHandle(handle);
        ret = true;
      }
    }
    return ret;
  }

  inline bool Release(CacheHandle* handle) {
    // 设置usage位，表示cache命中过
    handle->flags.fetch_or(kUsageBit, std::memory_order_relaxed);
    return Unref(handle);
  }

  CacheHandle* insert(K key, V value, uint32_t hash, bool hold_ref = false) {
    CacheHandle* handle = nullptr;
    std::lock_guard lock(mtx);
    if (not make_room()) {
      return handle;
    }
    // 回收的list如果有则复用
    if (!recycle_.empty()) {
      handle = recycle_.back();
      recycle_.pop_back();
    } else {
      // 没有则new一个
      list_.emplace_back();
      handle = &list_.back();
    }
    handle->key = key;
    handle->hash = hash;
    handle->value = value;
    uint32_t flags = hold_ref ? kInCacheBit + kOneRef : kInCacheBit;
    handle->flags.store(flags, std::memory_order_relaxed);
    auto iter = cache_.find(key);
    // 如果cache中存在，则对handle设置not in cahce
    if (iter != cache_.end()) {
      UnsetInCache(iter->second);
      cache_.erase(iter);
    }
    cache_.insert(key, handle);
    size_.fetch_add(1, std::memory_order_relaxed);
    return handle;
  }

  CacheHandle* find(K key, uint32_t hash) {
    auto iter = cache_.find(key);
    if (iter == cache_.end()) {
      return nullptr;
    }
    auto* handle = iter->second;
    if (!Ref(handle)) {
      return nullptr;
    }

    if (hash != handle->hash || key != handle->key) {
      Unref(handle);
      return nullptr;
    }
    return handle;
  }

  bool erase(K key, uint32_t hash) {
    std::lock_guard lock(mtx);
    bool erased = false;
    auto iter = cache_.find(key);
    if (iter != cache_.end()) {
      CacheHandle* handle = iter->second;
      cache_.erase(iter);
      erased = UnsetInCache(handle);
    }
    return erased;
  }

 protected:
  // 容量
  std::atomic<size_t> capacity;
  folly::ConcurrentHashMap<K, CacheHandle*> cache_;
  // handle内存管理
  std::deque<CacheHandle> list_;
  // 回收复用handle
  std::vector<CacheHandle*> recycle_;
  // size
  std::atomic<size_t> size_{0};
  size_t head_;

  mutable std::mutex mtx;
};

// 多路组相连cache
class SharedBlockCache {
 public:
  using CacheHandle = BlockCache<int32_t, BlockPtr, BlockPool>::CacheHandle;

  SharedBlockCache(size_t shard_num, size_t capacity)
      : shard_num(shard_num), mask(shard_num - 1) {
    // shard_num = 2^n
    assert(((shard_num) & (shard_num - 1)) == 0);
    shared_cache.reserve(shard_num);
    for (size_t i = 0; i < shard_num; i++) {
      shared_cache.emplace_back(
          new BlockCache<int32_t, BlockPtr, BlockPool>(capacity));
    }
  }
  ~SharedBlockCache() {
    for (auto ptr : shared_cache) {
      delete ptr;
    }
  }

  // insert and hold handle
  inline CacheHandle* insert_and_hold(int32_t key, BlockPtr value) {
    uint32_t hash = key & mask;
    auto* handle = shared_cache[hash]->insert(key, value, hash, true);
    return handle;
  }

  // insert and not hold handle
  inline bool insert(int32_t key, BlockPtr value) {
    uint32_t hash = key & mask;
    auto* handle = shared_cache[hash]->insert(key, value, hash, false);
    if (handle) return true;
    return false;
  }

  inline CacheHandle* find(int32_t key) {
    uint32_t hash = key & mask;
    return shared_cache[hash]->find(key, hash);
  }

  inline bool erase(int32_t key) {
    uint32_t hash = key & mask;
    return shared_cache[hash]->erase(key, hash);
  }

  inline bool release(CacheHandle* handle) {
    int32_t hash = handle->key & mask;
    return shared_cache[hash]->Release(handle);
  }

 protected:
  size_t shard_num;
  size_t mask;
  std::vector<BlockCache<int32_t, BlockPtr, BlockPool>*> shared_cache;
};

}  // namespace vamana