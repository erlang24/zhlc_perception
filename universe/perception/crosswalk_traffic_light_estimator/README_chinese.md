# crosswalk_traffic_light_estimator

## 目的

`crosswalk_traffic_light_estimator` 模块通过高精地图（HDMap）和已检测的车辆交通信号灯，估算人行横道的行人交通信号状态。（用于自动驾驶车辆通过人行横道时的决策）

## 输入/输出

### 输入

| 名称                                 | 类型                                             | 描述                  |
| ------------------------------------ | ------------------------------------------------ | -------------------- |
| `~/input/vector_map`                 | `autoware_auto_mapping_msgs::msg::HADMapBin`     | 矢量地图              |
| `~/input/route`                      | `autoware_planning_msgs::msg::LaneletRoute`      | 规划路径              |
| `~/input/classified/traffic_signals` | `tier4_perception_msgs::msg::TrafficSignalArray` | 已分类的交通信号灯    |

### 输出

| 名称                       | 类型                                             | 描述                     |
| -------------------------- | ------------------------------------------------ | ----------------------- |
| `~/output/traffic_signals` | `tier4_perception_msgs::msg::TrafficSignalArray` | 估算的行人交通信号灯状态 |

---

## 参数配置

| 参数名                          | 类型     | 描述                                                                                                                              | 默认值 |
| :------------------------------ | :------- | :------------------------------------------------------------------------------------------------------------------------------- | :----- |
| `use_last_detect_color`         | `bool`   | 当检测信号变为UNKNOWN时：<br>• `true`：若最后一次检测为GREEN/AMBER，行人信号保持RED<br>• `false`：仅依据最新检测结果估算信号 | true   |
| `last_detect_color_hold_time`   | `double` | 最后一次检测颜色的保持时间阈值（秒）                                                                                              | 2.0    |

---

## 算法原理

```plantuml
start
:订阅交通信号检测结果与高精地图;
:从HDMap提取人行横道车道段;
:提取与人行横道冲突的机动车道段;
:初始化non_red_lanelets(车道段集合);
if (最新检测结果为GREEN或AMBER?) then (是)
  :将车道段加入non_red_lanelets;
else (否)
  if (use_last_detect_color为true?) then (是)
    if (当前检测为UNKNOWN且上次检测为GREEN/AMBER?) then (是)
     :将车道段加入non_red_lanelets;
    endif
  endif
endif
if (non_red_lanelets中存在直行非红灯车道?) then (是)
  :关联行人信号设为RED;
else if (non_red_lanelets中同时存在左转/右转非红灯车道?) then (是)
  :关联行人信号设为RED;
else (否)
  :行人信号设为UNKNOWN;
endif
end