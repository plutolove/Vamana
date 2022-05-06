#pragma once
#include <libaio.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "boost/core/noncopyable.hpp"
#include "boost/lockfree/queue.hpp"
#include "common/define.h"
#include "fcntl.h"

namespace vamana {

struct Block {
  Block() {}
  Block(size_t start, size_t len) : start(start), len(len) {}
  // 获取数据ptr
  template <typename T>
  inline T* getPtr(size_t idx) {
    return reinterpret_cast<T*>(_data + idx);
  }

  // 获取Neighbor Size
  inline int32_t getNeighborSize(size_t idx) {
    int32_t size = -1;
    memcpy(&size, _data + idx, sizeof(int32_t));
    return size;
  }

  char _data[BLOK_SIZE];
  size_t start;
  size_t len;
  size_t idx;
};

class BlockReader : boost::noncopyable {
  using lockfree_queue = boost::lockfree::queue<io_context_t>;

 public:
  BlockReader(size_t ctx_size, const std::string& path);
  ~BlockReader() {
    if (fd != -1) {
      ::fcntl(fd, F_GETFD);
      ::close(fd);
    }
    int cnt = 0;
    while (not io_contexts.empty()) {
      io_context_t ctx;
      io_contexts.pop(ctx);
      io_destroy(ctx);
      cnt++;
    }
    std::cout << "io_destroy cnt: " << cnt << std::endl;
  }

  bool read(std::vector<Block>& blocks);

 protected:
  uint64_t file_sz;
  FileHandler fd = -1;
  lockfree_queue io_contexts;
};

}  // namespace vamana