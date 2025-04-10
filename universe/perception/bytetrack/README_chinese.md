# ByteTrack 字节追踪

## 目的

核心算法名为 `ByteTrack`，主要用于实现多目标跟踪。该算法通过关联几乎所有检测框（包括低置信度的检测框），有效减少漏检数量。

[演示视频](https://user-images.githubusercontent.com/3022416/225920856-745a3bb7-6b35-403d-87b0-6e5085952d70.mp4)

## 算法原理

### 引用文献

<!-- cspell:ignore Yifu Peize Jiang Dongdong Fucheng Weng Zehuan Xinggang -->

- Yifu Zhang, Peize Sun, Yi Jiang, Dongdong Yu, Fucheng Weng, Zehuan Yuan, Ping Luo, Wenyu Liu, 和 Xinggang Wang,  
  "ByteTrack: Multi-Object Tracking by Associating Every Detection Box"，发表于 ECCV 2022 [[论文链接](https://arxiv.org/abs/2110.06864)]
- 本包为从 [原代码库](https://github.com/ifzhang/ByteTrack/tree/main/deploy/TensorRT/cpp) 移植至 Autoware 的版本（ByteTrack 作者提供的 C++ 实现）

---

## 输入/输出

### bytetrack_node

#### 输入

| 名称       | 类型                                               | 描述                     |
| ---------- | -------------------------------------------------- | ------------------------ |
| `in/rect`  | `tier4_perception_msgs/DetectedObjectsWithFeature` | 带有2D边界框的检测对象   |

#### 输出

| 名称                     | 类型                                               | 描述                     |
| ------------------------ | -------------------------------------------------- | ------------------------ |
| `out/objects`            | `tier4_perception_msgs/DetectedObjectsWithFeature` | 带有2D边界框的检测对象   |
| `out/objects/debug/uuid` | `tier4_perception_msgs/DynamicObjectArray`         | 每个对象的唯一标识 (UUID) |

---

### bytetrack_visualizer

#### 输入

| 名称        | 类型                                                 | 描述                          |
| ----------- | ---------------------------------------------------- | ----------------------------- |
| `in/image`  | `sensor_msgs/Image` 或 `sensor_msgs/CompressedImage` | 进行目标检测的输入图像        |
| `in/rect`   | `tier4_perception_msgs/DetectedObjectsWithFeature`   | 带有2D边界框的检测对象        |
| `in/uuid`   | `tier4_perception_msgs/DynamicObjectArray`           | 每个对象的唯一标识 (UUID)     |

#### 输出

| 名称         | 类型                | 描述                          |
| ------------ | ------------------- | ----------------------------- |
| `out/image`  | `sensor_msgs/Image` | 绘制检测框和UUID的可视化图像   |

---

## 参数配置

### bytetrack_node

| 参数名                   | 类型  | 默认值 | 描述                          |
| ------------------------ | ----- | ------ | ----------------------------- |
| `track_buffer_length`    | int   | 30     | 判定轨迹段丢失的连续帧数阈值  |

### bytetrack_visualizer

| 参数名      | 类型  | 默认值 | 描述                          |
| ----------- | ----- | ------ | ----------------------------- |
| `use_raw`   | bool  | false  | 输入图像格式切换标志（原始图像或压缩图像） |

---

## 已知限制

- 当前版本暂未明确标注性能瓶颈和场景限制。

---

## 参考代码库

- [ByteTrack 官方仓库](https://github.com/ifzhang/ByteTrack)

---

## 许可证

- `lib` 目录下的代码修改自 [原代码库](https://github.com/ifzhang/ByteTrack/tree/72ca8b45d36caf5a39e949c6aa815d9abffd1ab5/deploy/TensorRT/cpp)，原代码遵循 MIT 许可证：
  
  > MIT License  
  > Copyright (c) 2021 Yifu Zhang  
  > （此处保留完整许可证文本，详见原文）

- 本移植包遵循 ​**Apache License 2.0**，与原始代码的 MIT 许可证并存。