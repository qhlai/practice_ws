roslaunch realsense2_camera rs_camera.launch
roslaunch vins vins_rviz.launch



alias ccc='catkin_make'
alias sss='source ./devel/setup.sh'
alias bbb='source ~/.bashrc'

rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml



$ sudo apt install libgoogle-glog-dev 

Also,
$ sudo apt purge libgoogle-glog-dev
$ sudo apt-get install libgflags-dev
$ sudo apt install libgoogle-glog-dev
$ sudo apt-get install protobuf-compiler libprotobuf-dev


locate gflag |grep /usr


roslaunch realsense2_camera rs_camera_d435i.launch align_depth:=true
rosrun vins vins_node ~/catkin_ws/src/slam/VINS-Fusion/config/euroc/d435i.yaml
rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/slam/VINS-Fusion/config/euroc/d435i.yaml
roslaunch vins vins_rviz.launch
roslaunch surfel_fusion vins_realsense.launch

sudo apt install ros-noetic-imu-filter-madgwick


rosrun vins vins_node  ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml


roslaunch jaka_ros_driver start.launch 



roslaunch realsense2_camera rs_camera_d435i.launch align_depth:=true

roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    unite_imu_method:="linear_interpolation" \
    enable_gyro:=true \
     enable_accel:=true
     
 rosrun imu_filter_madgwick imu_filter_node \
    _use_mag:=false \
    _publish_tf:=false \
    _world_frame:="enu" \
    /imu/data_raw:=/camera/imu \
    /imu/data:=/rtabmap/imu
    
    
roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu
    
    
[1.6054138696075382,1.5531737196619573,1.520179300016476,0.058488419009407225,-1.5090622400528064,-0.8175936680658081]
rosservice call /robot_driver/move_line "pose: [250,0,0,0,0,0]
has_ref: false
ref_joint: [0]
mvvelo: 10
mvacc: 200
mvtime: 0.0
mvradii: 0.0
coord_mode: 0
index: 0"

#go home
rosservice call /move_joint "pose: [-1.574390172958374,1.5622552633285522,-1.565582036972046,3.1364462375640874,1.5742380619049072,1.5742380619049072]
has_ref: false
ref_joint: [0]
mvvelo: 2.0
mvacc: 0.5
mvtime: 0.0
mvradii: 0.0
coord_mode: 0
index: 0"


rosservice call /jaka_driver/joint_move "pose: [1.5465844394635104, 1.4239856418533128, 0.0, 0.0, 0.0, 0.0]
has_ref: false
ref_joint: [0]
mvvelo: 2.0
mvacc: 0.5
mvtime: 0.0
mvradii: 0.0
coord_mode: 0
index: 0"


rosservice call /robot_driver/move_line [250,100,300,0,0,0] 50 2000 0 0


roscore
roslaunch jaka_ros_driver start.launch
rosrun control_msgs jaka5_server
roslaunch jaka5_config demo.launch
roslaunch jaka_control moveit.launch



roslaunch jaka_ros_driver start.launch
rosrun control_msgs jaka5_server
roslaunch jaka5_config demo.launch
roslaunch jaka_control moveit.launch


roslaunch handeye-calib aruco_start_realsense_sdk.launch
roslaunch handeye-calib online_hand_on_eye_calib_auto.launch

sudo apt install net-tools ifmetric

roslaunch jaka_ros_driver start.launch
rosrun control_msgs jaka5_server
roslaunch jaka5_config demo.launch
roslaunch jaka_control moveit.launch



roslaunch jaka_zu5_moveit_config demo.launch



roslaunch jaka_driver robot_start_launch.launch ip:=192.168.10.200

roslaunch jaka_driver robot_start_launch.launch ip:=192.168.10.200

roslaunch jaka_ros_driver start.launch
rosrun control_msgs jaka5_server
roslaunch jaka5_config demo.launch
roslaunch jaka_control calib.launch
roslaunch handeye-calib start_realsense_sdk.launch
rosrun rqt_reconfigure rqt_reconfigure          #short range
roslaunch handeye-calib aruco_single.launch
roslaunch handeye-calib online_hand_on_eye_calib_auto.launch

roslaunch handeye-calib start_realsense_sdk.launch
roslaunch handeye-calib aruco_double.launch



roslaunch jaka_control mapping.launch
roslaunch jaka_control calib.launch

rostopic echo /robot_driver/arm_pose





#############################
roslaunch jaka_driver robot_start_launch.launch ip:=192.168.10.200


###########
http://wiki.ros.org/rtabmap_ros/TutorialsNoetic/HandHeldMapping

roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    unite_imu_method:="linear_interpolation" \
    enable_gyro:=true \
     enable_accel:=true

rosrun imu_filter_madgwick imu_filter_node \
    _use_mag:=false \
    _publish_tf:=false \
    _world_frame:="enu" \
    /imu/data_raw:=/camera/imu \
    /imu/data:=/rtabmap/imu
    
    
roslaunch rtabmap_launch rtabmap.launch \
rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
depth_topic:=/camera/aligned_depth_to_color/image_raw \
rgb_topic:=/camera/color/image_raw \
camera_info_topic:=/camera/color/camera_info \
approx_sync:=false \
wait_imu_to_init:=true \
imu_topic:=/rtabmap/imu
#################
rosrun nodelet nodelet standalone rtabmap_util/point_cloud_xyz \
    _approx_sync:=false  \
    /depth/image:=/camera/depth/image_rect_raw \
    /depth/camera_info:=/camera/depth/camera_info \
    _decimation:=4

roslaunch rtabmap_launch rtabmap.launch\
    rtabmap_args:="\
      --delete_db_on_start \
      --Icp/VoxelSize 0.05 \
      --Icp/PointToPlaneRadius 0 \
      --Icp/PointToPlaneK 20 \
      --Icp/CorrespondenceRatio 0.2 \
      --Icp/PMOutlierRatio 0.65 \
      --Icp/Epsilon 0.005 \
      --Icp/PointToPlaneMinComplexity 0 \
      --Odom/ScanKeyFrameThr 0.7 \
      --OdomF2M/ScanMaxSize 15000 \
      --Optimizer/GravitySigma 0.3 \
      --RGBD/ProximityPathMaxNeighbors 1 \
      --Reg/Strategy 1" \
    icp_odometry:=true \
    scan_cloud_topic:=/cloud \
    subscribe_scan_cloud:=true \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu 


