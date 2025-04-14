# zhlc_perception
Autoware Partial Perception —— Comprehensive Road Test Part (Modified)

img_svg为精简后的流程（暂时）
![simplify_perception](img/simplify_perception.drawio.svg)

## 去除节点说明：

- <span style="color: cyan;">low_height_crop_box_filter</span>  
    - 不需要考虑低矮的障碍物进行了移除  

- <span style="color: cyan;">occupancy_grid_map_node</span> 和 <span style="color: cyan;">occupancy_grid_map_outlier_filter</span>
    - 用于生成和过滤占据栅格地图的，通常用于静态环境的建模和导航,主要任务是动态物体检测，进行了移除  

-  <span style="color: cyan;">voxel_based_compare_map_filter</span>
    - 不需要这种实时的地图比较（例如，在完全动态的环境中），可以移除这个节点  
    
-  <span style="color: cyan;">object_association_merger</span>
    - 将验证器输出和聚类输出进行合并，当前的节点图中没有明确的“验证器”节点，进行了移除

