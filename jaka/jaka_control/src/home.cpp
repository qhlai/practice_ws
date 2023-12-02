#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// #include "../include/jaka_jog_panel/main_window.hpp"


geometry_msgs::PoseStamped cam_pose;
sensor_msgs::JointState arm_pose_joint;
void aruco_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
// printf("test1=%f,test2=%d\n",msg->test1,msg->test2);
    // std::cout << "cam_pose1:" << msg->pose.position.x << ", "  << msg->pose.position.y << ", " << msg->pose.position.z << ", " 
    //  << msg->pose.orientation.w << ", "  << msg->pose.orientation.x << ", " << msg->pose.orientation.y << ", " << msg->pose.orientation.z
    //  <<std::endl;
     cam_pose=*msg;
}

void arm_pose_joint_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
// printf("test1=%f,test2=%d\n",msg->test1,msg->test2);
    // std::cout << "cam_pose1:" << msg->position[0]<< ", "  << msg->position[1]<< ", " << msg->position[2] << ", " 
    //  << msg->position[3] << ", "  << msg->position[4] << ", " << msg->position[5] 
    //  <<std::endl;
    //  cam_pose=*msg;
}

#define PI 3.1515926

int main(int argc, char **argv)
{
    //初始化节点
	ros::init(argc, argv, "moveit_cartesian_demo");
    ros::NodeHandle nh;
    //引入多线程
	ros::AsyncSpinner spinner(1);
    //开启多线程
	spinner.start();
    
    ros::Subscriber sub_cam_pose = nh.subscribe("/aruco_single/pose", 1, &aruco_pose_cb);
    ros::Subscriber sub_arm_pose_joint = nh.subscribe("/joint_states", 1, &arm_pose_joint_cb);

    //初始化需要使用move group控制的机械臂中的move_group group
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlannerId("RRTConnect"); // 选择运动规划器


    // // 存储关节角度的向量
    // std::vector<double> joint_values ={-1.574390172958374, 1.5622552633285522, -1.565582036972046, 3.1364462375640874, 1.5742380619049072, -0.7827801704406738} ;

    bool success =false;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // move_group.setJointValueTarget(joint_values); // 设置目标关节角度


    //获取终端link的名称
    std::string end_effector_link = move_group.getEndEffectorLink();
    std::cout << "end_effector_link:" << end_effector_link << std::endl;
    //设置目标位置所使用的参考坐标系
    // std::string reference_frame = "base_link";
    std::string reference_frame = "dummy";
    move_group.setPoseReferenceFrame(reference_frame);
 
    //当运动规划失败后，允许重新规划
    move_group.allowReplanning(true);
 
    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    move_group.setGoalPositionTolerance(0.001);
    move_group.setGoalOrientationTolerance(0.01);
 
    //设置允许的最大速度和加速度
    move_group.setMaxAccelerationScalingFactor(0.5);
    move_group.setMaxVelocityScalingFactor(0.2);
 
    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.move();
        ROS_INFO("move success");
        sleep(3);
    }
    else{ROS_ERROR("move fail");}

    sleep(1);  //停1s
 

     geometry_msgs::Pose start_pose = move_group.getCurrentPose(end_effector_link).pose;
    std::cout << "start_pose:" << start_pose.position.x << ", "  << start_pose.position.y << ", " << start_pose.position.z << ", " 
     << start_pose.orientation.w << ", "  << start_pose.orientation.x << ", " << start_pose.orientation.y << ", " << start_pose.orientation.z
     <<std::endl;

    

	ros::shutdown(); 
	return 0;
}