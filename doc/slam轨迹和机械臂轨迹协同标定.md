实验设备：
jaka5，realsense L515

使用方法

```shell
cd ~/catkin_ws
source ./devel/setup.sh
# 启动jaka的基本底层驱动，启动后需要等待几秒，等控制柜的灯为绿色
roslaunch jaka_ros_driver start.launch  
#发布机器人基坐标系下TCP位姿，10HZ发布
roslaunch handeye-calib arm2pose.launch


# 启动RTABMAP
# 略

# 记录数据集
rosbag record /rtabmap/odom /arm_pose

# evo库轨迹可视化与对齐
# https://github.com/MichaelGrupp/evo
# https://zhuanlan.zhihu.com/p/672731463
evo_traj bag 2023-12-13-16-23-50.bag  /rtabmap/odom --ref /arm_pose  -p --plot_mode=xyz --align --t_max_diff 500 --save_as_tum


evo_traj tum rtabmap_odom.tum --ref arm_pose.tum  -p --plot_mode=xyz  --align  --t_max_diff 500


#或者 处理数据集
python3 pc_ws/src/application/traj_match/scripts/trans.py
python3 pc_ws/src/application/traj_match/scripts/show.py

#保存的txt位姿为tum 格式，x y z qx qy qz qw
```

录制的数据集:
![[./asserts/2024-01-02 15-55-23屏幕截图.png]]

rosbag数据集处理和可视化，红色代表slam里程计，绿色代表机械臂获取的位姿

![[asserts/2024-01-02 15-52-20屏幕截图.png]]

### 样例1

xyz轨迹图
![[2024-01-02 15-32-26屏幕截图.png]]

xyz分表显示，slam里程计精度不足，无法很好跟随机械臂关节位姿
![[2024-01-02 15-32-43屏幕截图.png]]

### 样例2

![[2024-01-02 15-43-35屏幕截图.png]]

对上图处理异常值后
![[2024-01-02 15-46-29屏幕截图 1.png]]

![[2024-01-02 16-03-39屏幕截图.png]]

### 视觉slam定位精度

结论：通常情况下，视觉SLAM的精度可能不满足slam和机械臂轨迹融合标定问题

**均方根误差**( **RMSE**:Root Mean Square Error): 是观测值与真值偏差的平方和与观测次数m比值的平方根。 是用来衡量观测值同真值之间的偏差。表达式为：
![[Pasted image 20240102160600.png]]
**标准差**（**SD**: Standard Deviation ）: 是方差的算数平方根.是用来衡量一组数自身的离散程度
**SSE(和方差、误差平方和)**：The sum of squares due to error ,该统计 参数计算的是拟合 数据和原始数据对应点的误差的平方和
**STD(标准差） Standard Deviation**:一种量度数据分布的分散程度之标准，用以衡量数据值偏离算术平均值的程度。标准偏差越小，这些值偏离平均值就越少

以专注于定位且可复现性最佳的视觉SLAM，vins作为误差评估，比rtabmap精度更高,,更具代表性

### 精度分析

基于数据集Euroc MH_01_easy.bag数据集进行测试
未启动回环
最低误差为3cm，均值14厘米

```
#未启动回环
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	0.349640
      mean	0.144082
    median	0.140714
       min	0.034372
      rmse	0.154602
       sse	43.429475
       std	0.056053
```

启动回环

```
#未启动回环
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	0.141669
      mean	0.055870
    median	0.051059
       min	0.005496
      rmse	0.062941
       sse	4.163599
       std	0.028984



```

![[Pasted image 20240102161231.png]]

![[Pasted image 20240102161119.png]]
