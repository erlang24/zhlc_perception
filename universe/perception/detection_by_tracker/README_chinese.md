# detection_by_tracker （跟踪器检测）

## 目的

本模块通过将跟踪器信息反馈至检测模块，提升目标检测的持续性和稳定性。  
![检测优化示意图](image/purpose.svg)

核心功能：接收包含点云聚类结果的未知目标与跟踪器信息，通过优化未知目标的尺寸参数使其适配跟踪器，实现持续稳定检测。

## 算法原理

### 问题背景
基于欧几里得聚类等方法的形状拟合存在**过分割**​（Over-segmentation）与**欠分割**​（Under-segmentation）问题：

[![分割异常示例](image/segmentation_fail.png)](https://www.researchgate.net/figure/Examples-of-an-undersegmentation-error-top-and-an-oversegmentation-error-bottom-Each_fig1_304533062)  
_改编自[3]_

### 过分割处理策略
1. ​**目标合并** - 将跟踪器关联的多个未知目标合并为单一目标
2. ​**形状优化** - 利用跟踪器的角度、尺寸等信息作为先验进行形状拟合

### 欠分割处理策略
1. ​**检测评估** - 通过召回率与精确度的对比识别欠分割目标
2. ​**参数迭代** - 动态调整聚类参数生成细粒度聚类
3. ​**最优选取** - 选择与跟踪器交并比（IoU）最高的聚类结果

---

## 输入/输出

### 输入
| 名称                      | 类型                                                     | 描述          |
| ------------------------- | -------------------------------------------------------- | ------------- |
| `~/input/initial_objects` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | 初始未知目标  |
| `~/input/tracked_objects` | `tier4_perception_msgs::msg::TrackedObjects`             | 跟踪器信息    |

### 输出
| 名称        | 类型                                                  | 描述        |
| ----------- | ----------------------------------------------------- | ----------- |
| `~/output`  | `autoware_auto_perception_msgs::msg::DetectedObjects` | 优化后目标  |

---

## 参数配置
（当前版本暂无参数配置）

---

## 参考文献
[1] M. Himmelsbach 等. "基于自底向上/自顶向下检测的任意目标跟踪与分类", 2012.  
[2] Arya Senna Abdul Rachman. "自动驾驶中的3D-LiDAR多目标跟踪：城市道路不确定性下的多目标检测与跟踪", 2017.  
[3] David Held 等. "融合空间、时间与语义信息的实时3D分割概率框架", 2016.  

---

## 已知限制
<!-- 当前实现的假设与限制，例如：
- 依赖跟踪器的稳定性
- 复杂遮挡场景下可能失效 -->

---

## 未来扩展
<!-- 计划实现的功能：
- 多模态数据融合优化
- 动态参数自适应调整机制 -->