#include <cstddef>
#include <iostream>
#include <memory>

#include "block.h"
#include "common/define.h"
#include "fmt/format.h"
#include "libaio.h"

namespace vamana {

typedef struct io_event io_event_t;
typedef struct iocb iocb_t;

BlockReader::BlockReader(const std::string& path)
    : io_contexts(lockfree_queue(1024)) {
  int flags = O_DIRECT | O_RDONLY;
  fd = ::open(path.c_str(), flags);
  assert(fd != -1);
  /*
  // 初始化ctx_size个io context
  size_t cnt = 0;
  size_t tt = 0;
  while (true) {
    io_context_t ctx = 0;
    int ret = io_setup(MAX_EVENTS, &ctx);
    if (ret != 0) {
      std::cout << fmt::format("io setup error, ret: {}, idx: {}", ret, cnt)
                << std::endl;
      std::cout << "error msg: " << strerror(ret) << std::endl;
      continue;
    } else {
      std::cout << fmt::format("io setup success, ret: {}, idx: {}, error", ret,
                               cnt)
                << std::endl;
      std::cout << "error msg: " << strerror(ret) << std::endl;
    }
    io_contexts.push(ctx);
    cnt++;
    tt++;
    if (cnt >= ctx_size) break;
  }*/
}

bool BlockReader::read(std::vector<std::shared_ptr<Block>>& blocks) {
  io_context_t ctx = 0;
  if (not io_contexts.pop(ctx)) {
    int ret = io_setup(MAX_EVENTS, &ctx);
    if (ret != 0) {
      std::cout << fmt::format("io setup success, ret: {}, status: {}", ret,
                               strerror(ret))
                << std::endl;
    }
  }

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
      std::cerr << "io_submit() failed; returned " << ret
                << ", expected=" << batch_size << ", ernno=" << errno << " ="
                << ::strerror(-ret);
      std::cout << "ctx: " << ctx << "\n";
      return false;
    } else {
      ret = io_getevents(ctx, (int64_t)batch_size, (int64_t)batch_size,
                         evts.data(), nullptr);
      if (ret != (int64_t)batch_size) {
        std::cerr << "io_getevents() failed; returned " << ret
                  << ", expected=" << batch_size << ", ernno=" << errno << "="
                  << ::strerror(-ret) << "\n";
        return false;
      } else {
        break;
      }
    }
  }
  if (not io_contexts.push(ctx)) {
    io_destroy(ctx);
  }
  return true;
}

}  // namespace vamana