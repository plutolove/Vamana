cmake_minimum_required(VERSION 3.8)

project(Vamana)

set(CMAKE_CXX_STANDARD 17)

#OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(FATAL_ERROR "No OpenMP support")
endif()

find_package(folly REQUIRED)
find_package(Boost REQUIRED)
find_library(JemallocLib jemalloc)
find_package(gflags REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/src/)

file(GLOB_RECURSE SRC ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/src/*/*.cpp)

add_executable(Vamana ${SRC})
target_link_libraries(Vamana gflags ${Boost_LIBRARIES} aio ${JemallocLibs} ${FOLLY_LIBRARIES})
target_compile_options(Vamana PRIVATE -mavx2 -mavx -msse -msse2 -fPIC -fopenmp -fopenmp-simd)

add_executable(gen_data ${CMAKE_SOURCE_DIR}/tools/gen_data.cpp)
add_executable(read_data ${CMAKE_SOURCE_DIR}/tools/read_data.cpp)
add_executable(split_data ${CMAKE_SOURCE_DIR}/tools/split_data.cpp)