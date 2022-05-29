# Vamana
[![CI](https://github.com/plutolove/Vamana/actions/workflows/cmake.yml/badge.svg)](https://github.com/plutolove/Vamana/actions/workflows/cmake.yml)
## BUILD
```bash
#deps: folly, boost, jemalloc, gflags
git clone https://github.com/plutolove/Vamana.git --recursive 
cd Vamana/
mkdir build && cmake .. && make
```
## Index Format
#### Graph Index  
* first block: the number of point(size_t), dim(size_t), R(size_t), centroid_id(size_t)
* 4k per block: [vec: the number of neighbors: neighbors]

#### PQ Index  
* N(the number of point(size_t)) M(分段数size_t) sdim(每段的dim，size_t) cluster_num(每段的聚类数量size_t)
* 接下来M个block，每个block结构一样
  * cluster_num 个 sdim大小的vec，表示聚类中心的vec
  * cluster_num * cluster_num的数组，表示每个聚类中心的距离
* 接下来是N*M的uint8，表示每个点的pq code