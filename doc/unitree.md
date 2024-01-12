roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs


rosrun unitree_controller unitree_servo


rosrun unitree_controller unitree_external_force



a1机器狗具备三个计算单元，但是作为上层开发主要使用了tx2，位于机械狗前部的hdmi口位置





https://www.yuque.com/ironfatty/nly1un/ft1yvm

https://www.yuque.com/ironfatty/nly1un/fmocxa

## 如何连接网络

#### 方法1 使用usb网卡

测试了两张usb无线网卡，一张usb转rj45网卡，4种驱动都不太可用，手头没有免驱的无线网卡





#### 方法2 使用有线网口直连

打开终端，输入sudo gedit /etc/network/interfaces，根据提示输入密码，默认密码为123

增加配置信息时，注意有线端口的名字，这里以  eth0  举例，请根据实际情况来编辑。



增加的配置信息，可以是静态地址，也可以是动态分配（DHCP）的地址。



- 静态地址配置

  ```
  auto eth0:1
  iface eth0:1 inet static
  name Ethernet shangwang LAN card
  address 192.168.10.123
  netmask 255.255.255.0
  gateway 192.168.10.1
  ```

  

- 动态分配地址配置

  ```
  auto eth0:1
  iface eth0:1 inet dhcp
  
  ```



#### 建立路由，使之能连网

sudo route add default gw 192.168.10.1





### 启动lcm控制（highlevel）



source ./devel/setup.sh

#启动lcm server（负责使用udp跟运动控制单元通信） 和ros control（使之能够订阅ros topic）

roslaunch start start.launch

#仅启动lcm server

#roslaunch unitree_legged_real real.launch

#在ros control启动后，可使用键盘点动控制机械狗运动

roslaunch unitree_legged_real keyboard_control.launch

#lcm server 有时候会无法启动，使用如下指令找到进程pid，然后kill it

sudo lsof -i:8080
sudo kill -9 <PID>



#编译时缺少的一些库

sudo apt install  libactionlib-dev l libxmlrpcpp-dev  librosconsole-dev

#这个由遥控start键控制，查看是否允许机械狗运动

rostopic echo /dog_can_move

#直接给底盘发指令，向前以0.1m/s前进

rostopic pub /cmd_vel geometry_msgs/Twist -r 1 -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'



### slam建图

机械狗的相机为d435,并没有imu

解除图传对realsense的占用

killall RobotVisionSystem




melodic



    roslaunch realsense2_camera rs_camera.launch align_depth:=true


    roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false
