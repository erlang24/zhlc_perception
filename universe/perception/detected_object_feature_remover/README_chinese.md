# detected_object_feature_remover

## 目的

`detected_object_feature_remover` 是一个用于将消息类型从 `DetectedObjectWithFeatureArray` 转换为 `DetectedObjects` 的工具包，主要功能是移除检测对象中的特征字段。

## 实现原理

该节点直接移除 `DetectedObjectWithFeature` 中的特征字段（Feature），将其转换为标准检测对象格式 `DetectedObjects`。

---

## 输入/输出

### 输入

| 名称       | 类型                                                         | 描述                   |
| ---------- | ------------------------------------------------------------ | --------------------- |
| `~/input`  | `tier4_perception_msgs::msg::DetectedObjectWithFeatureArray` | 带特征字段的检测对象数组 |

### 输出

| 名称        | 类型                                                  | 描述               |
| ----------- | ----------------------------------------------------- | ----------------- |
| `~/output`  | `autoware_auto_perception_msgs::msg::DetectedObjects` | 标准检测对象数组   |

---

## 参数配置

本节点无配置参数。

---

## 已知限制

<!-- 
假设条件与限制说明，例如：
- 输入消息必须包含有效的 `feature` 字段
- 仅用于特定数据格式的向下兼容处理
-->