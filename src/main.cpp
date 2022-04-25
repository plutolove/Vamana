#include <iostream>

#include "common/distance_util.h"
#include "common/exception.h"
#include "fmt/format.h"

int main() {
  std::cout << fmt::format("xxxxxxxxxx {}", 2344) << std::endl;
  float x[] = {0.1, 0.34, 0.567};
  float y[] = {0.8, 0.6, 0.67};
  std::cout << fmt::format("dis: {}\n", ComputeCosineDistance_SSE(x, y, 3));
  return 0;
}