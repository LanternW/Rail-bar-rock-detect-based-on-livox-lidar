# Rail-bar-rock-detect-based-on-livox-lidar

A ros package


start the main node:
    roslaunch pc_measure highest.launch
    
It subscribes /livox/lidar topic
It publishes:
    /filtered_point_cloud : 经过统计滤波去噪点的原始数据
    /rotated_point_cloud  : 将点云中的“地面”旋转至与世界坐标系z = 0平面重合后的点云
    /highest_point_cloud  : 这个点云里就 1 个点， 检测出的轨条岩的最高点
