#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
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

// JAKAZuRobot robot;


#define PI 3.1515926

int main(int argc, char **argv)
{
	// // ros::AsyncSpinner spinner(1);
    // // //开启多线程
	// // spinner.start();
    // JointValue joints;
    // CartesianPose pose;
    // // joints[]
    // robot.kine_forward(&joints,&pose);
    ros::init(argc, argv, "kinematics_example");
    ros::NodeHandle nh;
    // 加载机器人模型
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    if (!robot_model)
    {
        ROS_ERROR("Failed to load robot model");
        return -1;
    }
    // 创建机器人状态对象
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    robot_state->setToDefaultValues();

    // 设置关节值
    std::vector<double> joint_values;
    // 设置关节值的具体数值

    // 获取运动学组
    const std::string& group_name = "manipulator";
    const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group_name);

    // 设置机器人状态的关节值
    robot_state->setJointGroupPositions(joint_model_group, joint_values);

    // // 计算正运动学解
    // const Eigen::Affine3d& end_effector_pose = robot_state->getGlobalLinkTransform("end_effector_link");

    // // 输出末端执行器的位姿信息
    // ROS_INFO_STREAM("End Effector Pose: \n" << end_effector_pose.matrix());

    // // 创建逆运动学对象
    // // kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader(robot_model);
    // kinematics::KinematicsPluginLoaderOptions kinematics_loader_options;
    // kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader(kinematics_loader_options);
    // // 加载运动学插件
    // kinematics::KinematicsBasePtr kinematics_solver = kinematics_loader.createKinematicsPlugin("your_kinematics_solver");

    // // 设置逆运动学目标位姿
    // geometry_msgs::Pose target_pose;
    // // 设置目标位姿的具体信息

    // // 设置逆运动学求解的超时时间
    // double timeout = 1.0;

    // // 计算逆运动学解
    // std::vector<double> ik_solution;
    // bool success = kinematics_solver->getPositionIK(target_pose, joint_values, ik_solution, timeout);

    // if (success)
    // {
    //     // 输出逆运动学解
    //     ROS_INFO("IK Solution: ");
    //     for (const auto& value : ik_solution)
    //     {
    //     ROS_INFO_STREAM(value << " ");
    //     }
    // }
    // else
    // {
    //     ROS_ERROR("IK computation failed");
    // }

	return 0;
}