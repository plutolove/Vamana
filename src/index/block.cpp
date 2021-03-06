#include <cassert>
#include <cstddef>
#include <iostream>
#include <memory>

#include "block.h"
#include "common/define.h"
#include "fmt/format.h"
#include "libaio.h"

namespace vamana {

BlockReader::BlockReader(const std::string& path) {
  int flags = O_DIRECT | O_RDONLY;
  fd = ::open(path.c_str(), flags);
  assert(fd != -1);
}

bool BlockReader::read(std::vector<BlockPtr>& blocks, io_context_t ctx) {
  size_t iter_num = DIV_ROUND_UP(blocks.size(), MAX_EVENTS);
  size_t idx = 0;
  for (size_t iter_id = 0; iter_id < iter_num; ++iter_id) {
    size_t batch_size = std::min((size_t)blocks.size() - (iter_id * MAX_EVENTS),
                                 (size_t)MAX_EVENTS);

    std::vector<iocb_t*> cbs(batch_size, nullptr);
    std::vector<io_event_t> evts(batch_size);
    std::vector<struct iocb> cb(batch_size);
    for (size_t i = 0; i < batch_size and idx < blocks.size(); ++i) {
      io_prep_pread(cb.data() + i, fd, blocks[idx]->data, blocks[idx]->len,
                    blocks[idx]->start);
      ++idx;
    }

    for (uint64_t i = 0; i < batch_size; i++) {
      cbs[i] = cb.data() + i;
    }

    int64_t ret = io_submit(ctx, (int64_t)batch_size, cbs.data());
    if (ret != (int64_t)batch_size) {
      std::cout << fmt::format("io_submit() failed, ret: {}, status: {}", ret,
                               strerror(-ret))
                << std::endl;
      return false;
    } else {
      ret = io_getevents(ctx, (int64_t)batch_size, (int64_t)batch_size,
                         evts.data(), nullptr);
      if (ret != (int64_t)batch_size) {
        std::cout << fmt::format("io_getevents() failed, ret: {}, status: {}",
                                 ret, strerror(-ret))
                  << std::endl;
        return false;
      }
    }
  }
  return true;
}

bool BlockReader::read(BlockPtr& block, io_context_t ctx) {
  std::vector<iocb_t*> cbs(1, nullptr);
  std::vector<io_event_t> evts(1);
  std::vector<struct iocb> cb(1);

  io_prep_pread(cb.data(), fd, block->data, block->len, block->start);

  cbs[0] = cb.data();

  int64_t ret = io_submit(ctx, (int64_t)1, cbs.data());
  if (ret != (int64_t)1) {
    std::cout << fmt::format("io_submit() failed, ret: {}, status: {}", ret,
                             strerror(-ret))
              << std::endl;
    return false;
  } else {
    ret = io_getevents(ctx, (int64_t)1, (int64_t)1, evts.data(), nullptr);
    if (ret != (int64_t)1) {
      std::cout << fmt::format("io_getevents() failed, ret: {}, status: {}",
                               ret, strerror(-ret))
                << std::endl;
      return false;
    }
  }
  return true;
}

}  // namespace vamana