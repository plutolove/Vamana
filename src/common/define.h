#pragma once
#include <cstdint>

#define ALWAYS_INLINE inline __attribute__((always_inline))

#define ROUND_UP(X, Y) \
  ((((uint64_t)(X) / (Y)) + ((uint64_t)(X) % (Y) != 0)) * (Y))

#define DIV_ROUND_UP(X, Y) (((uint64_t)(X) / (Y)) + ((uint64_t)(X) % (Y) != 0))

#define ROUND_DOWN(X, Y) (((uint64_t)(X) / (Y)) * (Y))

const int BLOK_SIZE = 4096;
const int MAX_EVENTS = 1024;
const int TRAINING_SET_SIZE = 100000;

using FileHandler = int;