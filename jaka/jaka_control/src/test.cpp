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


#define PI 3.1515926
int main(int argc, char **argv)
{
    //初始化节点
	ros::init(argc, argv, "moveit_cartesian_demo");
    //引入多线程
	ros::AsyncSpinner spinner(10);
    //开启多线程
	spinner.start();
 
    //初始化需要使用move group控制的机械臂中的move_group group
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlannerId("EST"); // 选择运动规划器



    // 0.220989, 0.692915, 0.541767, 0.693106, 0.478885, -0.535783, -0.0566535
    // 0.224007, 0.74447, 0.284525, 0.681414, 0.535849, -0.498534, 0.00234037
    // -0.587954, 0.591474, 0.494112, 0.388549, 0.73261, 0.0939782, 0.550891


    // // 存储关节角度的向量
    // std::vector<double> joint_values ={-1.574390172958374, 1.5622552633285522, -1.565582036972046, 3.1364462375640874, 1.5742380619049072, -0.7827801704406738} ;
    geometry_msgs::Pose  curr_pose, tmp;
    bool success =false;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // move_group.setJointValueTarget(joint_values); // 设置目标关节角度

    // // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    // {
    //     move_group.move();
    //     ROS_INFO("移动成功！");
    // }
    // else{ROS_ERROR("无法完成移动！");}
    // -0.0403016, 0.732443, 0.590482, 0.66653, 0.637603, -0.321447, 0.214177
    // -0.0403016, 0.732443, 0.590482, 0.66653, 0.637603, -0.321447, 0.214177
    // -0.0403016, 0.732443, 0.590482, 0.66653, 0.637603, -0.321447, 0.214177
    // -0.0403016, 0.732443, 0.590482, 0.66653, 0.637603, -0.321447, 0.214177

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

    sleep(30);  //停1s
     curr_pose=move_group.getCurrentPose(end_effector_link).pose;



    std::cout << "start_pose1:" << curr_pose.position.x << ", "  << curr_pose.position.y << ", " << curr_pose.position.z << ", " 
     << curr_pose.orientation.w << ", "  << curr_pose.orientation.x << ", " << curr_pose.orientation.y << ", " << curr_pose.orientation.z
     <<std::endl;

    tmp=curr_pose;
    Eigen::Quaterniond desired_orientation;  
    Eigen::Vector3d reference_vector(0.0, 0.0, -1.0);  // 参考向量，末端应该指向该向量的方向 
    desired_orientation.setFromTwoVectors(reference_vector, reference_vector);
    // tmp.orientation.w = desired_orientation.w();
    // tmp.orientation.x = desired_orientation.x();
    // tmp.orientation.y = desired_orientation.y();
    // tmp.orientation.z = desired_orientation.z();
    tmp.position.y-=0.2;
    move_group.setPoseTarget(tmp);


    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.move();
        ROS_INFO("success");
    }
    else{ROS_ERROR("fail");}
    sleep(30);
  

    curr_pose=move_group.getCurrentPose(end_effector_link).pose;

    std::cout << "start_pose1:" << curr_pose.position.x << ", "  << curr_pose.position.y << ", " << curr_pose.position.z << ", " 
     << curr_pose.orientation.w << ", "  << curr_pose.orientation.x << ", " << curr_pose.orientation.y << ", " << curr_pose.orientation.z
     <<std::endl;

 
	ros::shutdown(); 
	return 0;
}