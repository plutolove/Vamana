cmake_minimum_required(VERSION 3.8)

project(Vamana)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost 1.67 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/src/)
add_subdirectory(${CMAKE_SOURCE_DIR}/third_deps/fmt)
include_directories(${CMAKE_SOURCE_DIR}/third_deps/fmt/include)

file(GLOB_RECURSE SRC ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/src/*/*.cpp)

add_executable(Vamana ${SRC})
target_link_libraries(Vamana ${Boost_LIBRARIES} fmt-header-only)
target_compile_options(Vamana PRIVATE -mavx2 -mavx -msse -msse2 -fPIC)

add_executable(gen_data ${CMAKE_SOURCE_DIR}/tools/gen_data.cpp)
add_executable(read_data ${CMAKE_SOURCE_DIR}/tools/read_data.cpp)
add_executable(fvec ${CMAKE_SOURCE_DIR}/tools/fvec.cpp)