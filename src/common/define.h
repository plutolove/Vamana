#pragma once
#include <cstdint>

#define ALWAYS_INLINE inline __attribute__((always_inline))

#define ROUND_UP(X, Y) \
  ((((uint64_t)(X) / (Y)) + ((uint64_t)(X) % (Y) != 0)) * (Y))

#define DIV_ROUND_UP(X, Y) (((uint64_t)(X) / (Y)) + ((uint64_t)(X) % (Y) != 0))

#define ROUND_DOWN(X, Y) (((uint64_t)(X) / (Y)) * (Y))

const int BLOCK_SIZE = 4096;
const int MAX_EVENTS = 1024;

using FileHandler = int;

typedef struct io_event io_event_t;
typedef struct iocb iocb_t;