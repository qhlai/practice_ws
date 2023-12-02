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

geometry_msgs::Pose arm2cam(geometry_msgs::Pose& pose, Eigen::Isometry3d ee_to_camera){
    geometry_msgs::Pose pose1;


}

geometry_msgs::Pose cam2arm(geometry_msgs::Pose& pose, Eigen::Isometry3d ee_to_camera){
    geometry_msgs::Pose pose1;


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
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;


    for(int i = 0; i < 6; i++)
    {
        calib_cmd.position.push_back(0); // write data into standard ros msg
    }

    //获取终端link的名称
    std::string end_effector_link = move_group.getEndEffectorLink();
    std::cout << "end_effector_link:" << end_effector_link << std::endl;
    //设置目标位置所使用的参考坐标系
    // std::string reference_frame = "base_link";
    std::string reference_frame = "world";
    move_group.setPoseReferenceFrame(reference_frame);
    //当运动规划失败后，允许重新规划
    move_group.allowReplanning(true);
    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setStartStateToCurrentState();
    //设置允许的最大速度和加速度
    move_group.setMaxAccelerationScalingFactor(0.5);
    move_group.setMaxVelocityScalingFactor(0.2);
 
    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    arm_move(move_group, my_plan);




    ee_to_camera = Eigen::Isometry3d::Identity();
    // 设置位移和旋转
    // {
    //     Eigen::Vector3d translation(0.0475516, 0.0273706, -0.0319397);
    //     // w x y z
    //     Eigen::Quaterniond quaternion(0.010810580144438161, -0.9280046596171926, 0.3717981394954345, 0.021368821994964145);
    //     ee_to_camera.translation() = translation;
    //     ee_to_camera.rotate(quaternion);
    // }


    ROS_INFO("next move");
    float bias_pre_x=-0.2;
    float bias_pre_y=0;
    float bias_pre_z=-0.10;

    // geometry_msgs::Pose target_pose; // 设置目标姿势

    // // 设置目标姿势的位置和朝向

    // 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = move_group.getCurrentPose(end_effector_link).pose;
    
    // float joint_bias = 0.02;
    // if(abs(start_pose.position.x-1.574390172958374)<joint_bias && )

    // 创建Eigen向量表示中心点的位置
    Eigen::Vector3d target_position(start_pose.position.x+bias_pre_x, start_pose.position.y+bias_pre_y, 0.00);

    std::cout << "start_pose:" << start_pose.position.x << ", "  << start_pose.position.y << ", " << start_pose.position.z << ", " 
     << start_pose.orientation.w << ", "  << start_pose.orientation.x << ", " << start_pose.orientation.y << ", " << start_pose.orientation.z
     <<std::endl;

    //初始化路径点向量
	// std::vector<geometry_msgs::Pose> waypoints;
    //将初始位姿加入路点列表
	// waypoints.push_back(start_pose);    


    float r = 0.15;
    int points_num =40;
    geometry_msgs::Pose tmp;
    geometry_msgs::Pose current_pose;
     
    {
        Eigen::Vector3d reference_vector(0.0, 0.0, -1.0);  // 参考向量，末端应该指向该向量的方向
        Eigen::AngleAxisd rotation_vector(0, Eigen::Vector3d(0, 0,-1)); 
        // desired_orientation=reference_vector;
        Eigen::Quaterniond desired_orientation(rotation_vector); 
        // desired_orientation.setFromTwoVectors(reference_vector, reference_vector); 
        tmp=start_pose;
        // tmp.orientation=desired_orientation;
        tmp.orientation.w = desired_orientation.w();
        tmp.orientation.x = desired_orientation.x();
        tmp.orientation.y = desired_orientation.y();
        tmp.orientation.z = desired_orientation.z();
    }
    tmp.position.x+=bias_pre_x;
    tmp.position.y+=bias_pre_y;
    tmp.position.z+=bias_pre_z;
    move_group.setPoseTarget(tmp);
    std::cout << "ori:" << tmp.orientation.w << ", "<< tmp.orientation.x << ", "<< tmp.orientation.y << ", "<< tmp.orientation.z << std::endl;
    arm_move(move_group, my_plan);
    sleep(3);
    // std::cout << "test" <<  std::endl;
    // tmp.position.x+=0.2;
    // arm_move(move_group, my_plan);
    // sleep(3);  
    // std::cout << "test" <<  std::endl;
    // tmp.position.y-=0.2;
    // arm_move(move_group, my_plan);
    // sleep(3);  
#if 0
    std::cout << "rough calib start" <<  std::endl;
    // 相机归中
    geometry_msgs::PoseStamped cam_pose_last, cam_pose_x_move, cam_pose_y_move;
    double move_len=0.01;
    double move_step_interval=0.01;
    geometry_msgs::Pose pre_calib_pose;
    while(abs(cam_pose.pose.position.x) > 0.02  && (abs(cam_pose.pose.position.x)+abs(cam_pose.pose.position.y)+abs(cam_pose.pose.position.z))!=0){
        // geometry_msgs::Pose pre_calib_pose;
        cam_pose_last=cam_pose;
        pre_calib_pose=tmp;
        pre_calib_pose.position.x+=move_len;
        move_group.setPoseTarget(tmp);
        
        arm_move(move_group, my_plan);
        // sleep(1);
        if(abs(cam_pose.pose.position.x) < abs(cam_pose_last.pose.position.x) || abs(cam_pose.pose.position.y) < abs(cam_pose_last.pose.position.y)){
            std::cout << "better" << std::endl;
            move_len+=move_step_interval;
        }else{
            std::cout << "worse" << std::endl;
            // move_len=-move_len/2;
            move_step_interval=-move_step_interval;
        }
        std::cout <<move_len<< " cam_pose_x:" <<cam_pose.pose.position.x<<", "<<cam_pose.pose.position.y<<", "<<cam_pose.pose.position.z<< std::endl;
    }
    sleep(2);
    move_len=0.01;
    while(abs(cam_pose.pose.position.y) > 0.02  && (abs(cam_pose.pose.position.x)+abs(cam_pose.pose.position.y)+abs(cam_pose.pose.position.z))!=0){
        // geometry_msgs::Pose pre_calib_pose;
        cam_pose_last=cam_pose;
        pre_calib_pose=tmp;
        pre_calib_pose.position.y+=move_len;
        move_group.setPoseTarget(tmp);
        arm_move(move_group, my_plan);
        // sleep(1);
        if(abs(cam_pose.pose.position.y) < abs(cam_pose_last.pose.position.y)){
            std::cout << "better" << std::endl;
            move_len+=move_step_interval;
        }else{
            std::cout << "worse" << std::endl;
            // move_len=-move_len/2;
            move_step_interval=-move_step_interval;
        }
        std::cout <<move_len<< " cam_pose_y:" <<cam_pose.pose.position.x<<", "<<cam_pose.pose.position.y<<", "<<cam_pose.pose.position.z<< std::endl;
    }
    // geometry_msgs::PoseStamped cam_pose_last, cam_pose_x_move, cam_pose_y_move;
    // cam_pose_last = cam_pose;
    // tmp.position.x+=0.1;
    // move_group.setPoseTarget(tmp);
    // arm_move(move_group, my_plan);
    // cam_pose_x_move = cam_pose;
    // tmp.position.x+=-0.1;
    // tmp.position.y+=0.1;
    // move_group.setPoseTarget(tmp);
    // cam_pose_y_move = cam_pose;
    // sleep(3);
    std::cout << "rough calib done" <<  std::endl;
    // if()
    // waypoints.push_back(tmp);
    // tmp=pre_calib_pose;
    target_position[0]=pre_calib_pose.position.x;
    target_position[1]=pre_calib_pose.position.y;    
#endif
    current_pose =  move_group.getCurrentPose(end_effector_link).pose;
    geometry_msgs::Pose mission_start_pose =  move_group.getCurrentPose(end_effector_link).pose;
    // target_position[2]=pre_calib_pose-pre_calib_pose.position.z;
    for(int i = 0 ; i < points_num; ++i){
        tmp=mission_start_pose;

        current_pose =  move_group.getCurrentPose(end_effector_link).pose;
        // std::vector<double> current_joint_values = group.getCurrentJointValues();

        tmp.position.x += r * cos(2*PI/points_num*i);
        tmp.position.y += r * sin(2*PI/points_num*i);
        tmp.position.z += -0.05;
        // tmp.position.x+=bias_pre_x;
        // tmp.position.y+=bias_pre_y;
        // tmp.position.z+=bias_pre_z;


        Eigen::Vector3d current_position(tmp.position.x, tmp.position.y, tmp.position.z);
        // 计算机械臂的面朝方向向量
        Eigen::Vector3d direction_vector = (target_position - current_position).normalized();
#if 0
        Eigen::Vector3d reference_vector(0.0, 0.0, -1.0);  // 参考向量，末端应该指向该向量的方向
        // Eigen::AngleAxisd rotation_vector(0, direction_vector); 
        // desired_orientation=reference_vector;
        // Eigen::Quaterniond desired_orientation(rotation_vector); 
        Eigen::Quaterniond desired_orientation;  
        desired_orientation.setFromTwoVectors(reference_vector, direction_vector);
#else
        Eigen::Quaterniond desired_orientation(1, direction_vector[0], direction_vector[1], direction_vector[2]);
        desired_orientation.normalize();
        // // tmp.orientation=desired_orientation;
        // tmp.orientation.w = quaternion.w();
        // tmp.orientation.x = quaternion.x();
        // tmp.orientation.y = quaternion.y();
        // tmp.orientation.z = quaternion.z();
#endif
        // std::cout << "d_vector:" << direction_vector[0] <<", "<< direction_vector[1]  <<", "<< direction_vector[2] << std::endl;
        // 计算朝向的四元数

        // desired_orientation.setFromTwoVectors(-Eigen::Vector3d::UnitZ(), direction_vector);

        Eigen::Quaterniond final_orientation =  desired_orientation;
        final_orientation.normalize();
        tmp.orientation.w = final_orientation.w();
        tmp.orientation.x = final_orientation.x();
        tmp.orientation.y = final_orientation.y();
        tmp.orientation.z = final_orientation.z();
        std::cout << "ori:" << final_orientation.w() << ", "<< final_orientation.x() << ", "<< final_orientation.y() << ", "<< final_orientation.z() << std::endl;
        move_group.setPoseTarget(tmp);
        arm_move(move_group, my_plan);
        sleep(1);
        
        calib_cmd.position[0]=1.0;
        calib_cmd_pub.publish(calib_cmd);
        calib_cmd.position[0]=0;
        sleep(1);
    }
	calib_cmd.position[0]=2.0;
    calib_cmd_pub.publish(calib_cmd);
    calib_cmd.position[0]=0;

    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    move_group.move();
    sleep(10);
    calib_cmd.position[0]=3.0;
    calib_cmd_pub.publish(calib_cmd);
    calib_cmd.position[0]=0;
    // move_group.move();
    // sleep(1);
    ros::Rate loop_rate(0.5);
    while (ros::ok())
    {
        loop_rate.sleep();
    }
	ros::shutdown(); 
	return 0;
}