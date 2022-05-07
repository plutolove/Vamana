#pragma once
#include <libaio.h>

#include <boost/align/detail/aligned_alloc_posix.hpp>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "boost/core/noncopyable.hpp"
#include "boost/lockfree/queue.hpp"
#include "boost/noncopyable.hpp"
#include "common/define.h"
#include "fcntl.h"

namespace vamana {

struct Block : boost::noncopyable {
  Block() {}
  Block(size_t start, size_t len) : start(start), len(len) {
    data = reinterpret_cast<char*>(
        boost::alignment::aligned_alloc(512, BLOK_SIZE));
  }

  ~Block() {
    if (data) boost::alignment::aligned_free(data);
  }

  // 获取数据ptr
  template <typename T>
  inline T* getPtr(size_t idx) {
    return reinterpret_cast<T*>(data + idx);
  }

  // 获取Neighbor Size
  inline int32_t getNeighborSize(size_t idx) {
    int32_t size = -1;
    memcpy(&size, data + idx, sizeof(int32_t));
    return size;
  }

  char* data = nullptr;
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

  bool read(std::vector<std::shared_ptr<Block>>& blocks);

 protected:
  uint64_t file_sz;
  FileHandler fd = -1;
  lockfree_queue io_contexts;
};

}  // namespace vamana