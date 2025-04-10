# euclidean_cluster

## 目的

本模块提供两种点云聚类算法，用于目标物体分割与分类：
- ​**欧几里得聚类** (`euclidean_cluster`)
- ​**体素化欧几里得聚类** (`voxel_grid_based_euclidean_cluster`)

## 算法原理

### 欧几里得聚类
基于PCL库的`pcl::EuclideanClusterExtraction`算法实现，[官方文档](https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html)

### 体素化欧几里得聚类
1. ​**体素降采样** - 使用`pcl::VoxelGrid`计算每个体素的质心
2. ​**质心聚类** - 对质心点云执行欧几里得聚类
3. ​**原始点分配** - 根据质心聚类结果分配原始点云

---

## 输入/输出

### 输入
| 名称       | 类型                            | 描述        |
| ---------- | ------------------------------- | ---------- |
| `input`    | `sensor_msgs::msg::PointCloud2` | 输入点云    |

### 输出
| 名称              | 类型                                                     | 描述                  |
| ----------------- | -------------------------------------------------------- | -------------------- |
| `output`          | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | 聚类结果（带特征）    |
| `debug/clusters`  | `sensor_msgs::msg::PointCloud2`                          | 可视化彩色聚类点云    |

---

## 参数配置

### 欧几里得聚类
| 参数名             | 类型    | 描述                                  | 
| ------------------ | ------- | ------------------------------------ |
| `use_height`       | bool    | 启用Z轴维度参与聚类                   |
| `min_cluster_size` | int     | 最小聚类点数阈值（低于此值将被过滤）  |
| `max_cluster_size` | int     | 最大聚类点数阈值（高于此值将被过滤）  |
| `tolerance`        | float   | 空间聚类容差（L2欧氏空间距离，单位：米） |

### 体素化欧几里得聚类
| 参数名                     | 类型    | 描述                                  |
| ------------------------- | ------- | ------------------------------------ |
| `use_height`              | bool    | (暂未实现) 启用Z轴维度参与聚类        |
| `min_cluster_size`        | int     | 最小聚类点数阈值                      |
| `max_cluster_size`        | int     | 最大聚类点数阈值                      |
| `tolerance`               | float   | 空间聚类容差（单位：米）               |
| `voxel_leaf_size`         | float   | XY平面体素尺寸（单位：米）             |
| `min_points_number_per_voxel` | int | 单个体素的最小点数阈值               |

---

## 已知限制
<!-- 
当前实现假设：
- 适用于静态场景分析，动态物体快速移动时可能产生分割错误
- 未考虑传感器盲区问题
-->

---

## 性能特征
<!-- 
### 时间复杂度
- 欧几里得聚类: O(N logN)
- 体素化聚类: O(M logM) (M为降采样后的质心点数)

### 处理耗时
Intel i7-11800H @10万点云：
- 欧几里得聚类: 8ms 
- 体素化聚类: 12ms (含降采样步骤)
-->

---

## 参考文献
[1] PCL官方文档 - 聚类提取: https://pcl.readthedocs.io  
[2] 体素网格算法说明: https://pointclouds.org/documentation/tutorials/voxel_grid.html

---

## 未来扩展
- [ ] 实现体素化聚类的`use_height`参数（当前版本Z轴维度未启用）
- [ ] 支持多线程加速处理
- [ ] 动态参数自适应调整机制