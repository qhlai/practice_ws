#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
// #include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_state/attached_body.h>

#include "robot_msgs/RobotMsg.h"

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/JointState.h"
// #include "../include/jaka_jog_panel/main_window.hpp"


geometry_msgs::PoseStamped cam_pose;
sensor_msgs::JointState arm_pose_joint;
sensor_msgs::JointState arm_pose_joint_last;
robot_msgs::RobotMsg robot_state_msg;
ros::Publisher pose_pub;
Eigen::Isometry3d ee_to_camera;
void robot_states_cb(const robot_msgs::RobotMsg::ConstPtr& msg)
{
     robot_state_msg=*msg;
}

void aruco_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     cam_pose=*msg;
}

void arm_pose_joint_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    arm_pose_joint=*msg;
}

void pub_curr_pose(moveit::planning_interface::MoveGroupInterface& move_group){
    std::string end_effector_link = move_group.getEndEffectorLink();
    geometry_msgs::Pose current_pose =  move_group.getCurrentPose(end_effector_link).pose;
    std::cout << "current_pose:" << current_pose.position.x << ", "  << current_pose.position.y << ", " << current_pose.position.z << ", " 
     << current_pose.orientation.w << ", "  << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z
     <<std::endl;


    // ee_to_camera

    geometry_msgs::PoseStamped pose;
    pose.pose = current_pose;

    Eigen::Vector3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    // 将geometry_msgs::Pose中的旋转部分转换为Eigen中的四元数
    Eigen::Quaterniond quaternion(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);

    // 创建转移矩阵
    Eigen::Isometry3d transformationMatrix = Eigen::Isometry3d::Identity();
    transformationMatrix.translation() = translation;
    transformationMatrix.linear() = quaternion.toRotationMatrix();

    Eigen::Isometry3d final_tf =transformationMatrix*ee_to_camera;
    // Eigen::Isometry3d final_tf =transformationMatrix;

    // geometry_msgs::Pose pose;
    pose.pose.position.x = final_tf.translation().x();
    pose.pose.position.y = final_tf.translation().y();
    pose.pose.position.z = final_tf.translation().z();
    Eigen::Quaterniond quaternion1(final_tf.linear());
    pose.pose.orientation.w = quaternion1.w();
    pose.pose.orientation.x = quaternion1.x();
    pose.pose.orientation.y = quaternion1.y();
    pose.pose.orientation.z = quaternion1.z();

     pose_pub.publish(pose);
}
void arm_move(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::MoveGroupInterface::Plan& plan){
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.move();
        ROS_INFO("move cmd send");
        // sleep(3);
    }
    else{ROS_ERROR("move fail");}
    while(1){
        if(robot_state_msg.state == 0){
            break;
        }
    }
    pub_curr_pose(move_group);
    std::cout << "move done" <<std::endl;
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
    ros::Subscriber sub_robot_state_pose = nh.subscribe("/l_arm_controller/robot_driver/robot_states", 10, &robot_states_cb);
    ros::Subscriber sub_arm_pose_joint = nh.subscribe("/joint_states", 1, &arm_pose_joint_cb);
    ros::Publisher calib_cmd_pub = nh.advertise<sensor_msgs::JointState>("/calib_cmd", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_moveit", 1);

    //初始化需要使用move group控制的机械臂中的move_group group
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlannerId("EST"); // 选择运动规划器

    // // 存储关节角度的向量
    // std::vector<double> joint_values ={-1.574390172958374, 1.5622552633285522, -1.565582036972046, 3.1364462375640874, 1.5742380619049072, -0.7827801704406738} ;
    geometry_msgs::Pose  curr_pose;
    geometry_msgs::Pose  start_pose;
    sensor_msgs::JointState calib_cmd; 
    
    bool success =false;

    ee_to_camera= Eigen::Isometry3d::Identity();
  
    // 控制机械臂先回到初始化位置
    // move_group.setNamedTarget("home");
    // move_group.move();
    // sleep(10);
    // calib_cmd.position[0]=3.0;
    // calib_cmd_pub.publish(calib_cmd);
    // calib_cmd.position[0]=0;
    // move_group.move();
    // sleep(1);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {


        while(1){
            if(robot_state_msg.state == 0){
                break;
            }
        }

        std::string end_effector_link = move_group.getEndEffectorLink();
        geometry_msgs::Pose current_pose =  move_group.getCurrentPose(end_effector_link).pose;
        std::cout << "current_pose:" << current_pose.position.x << ", "  << current_pose.position.y << ", " << current_pose.position.z << ", " 
        << current_pose.orientation.w << ", "  << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z
        <<std::endl;

        pub_curr_pose(move_group);
        loop_rate.sleep();
    }
	ros::shutdown(); 
	return 0;
}