#pragma once
#include <libaio.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

#include "boost/core/noncopyable.hpp"
#include "boost/noncopyable.hpp"
#include "common/define.h"
#include "fcntl.h"

namespace vamana {

struct Block : boost::noncopyable {
  Block()
      : start(-1),
        len(BLOCK_SIZE),
        data(reinterpret_cast<char*>(std::aligned_alloc(512, BLOCK_SIZE))) {}
  Block(size_t start, size_t len) : start(start), len(len) {
    data = reinterpret_cast<char*>(std::aligned_alloc(512, BLOCK_SIZE));
  }

  ~Block() {
    if (data) free(data);
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

  size_t start;
  size_t len;
  char* data = nullptr;
  size_t idx;
};

using BlockPtr = Block*;

class BlockReader : boost::noncopyable {
 public:
  BlockReader(const std::string& path);
  ~BlockReader() {
    if (fd != -1) {
      ::fcntl(fd, F_GETFD);
      ::close(fd);
    }
  }

  // 读取多个block
  bool read(std::vector<BlockPtr>& blocks, io_context_t ctx);
  // 读取一个block
  bool read(BlockPtr& block, io_context_t ctx);

 protected:
  uint64_t file_sz;
  FileHandler fd = -1;
};

}  // namespace vamana