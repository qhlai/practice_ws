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

float target_x;
float target_y;
float target_z;
#define PI 3.1515926
void TarCB(const geometry_msgs::Pose& tar){
    target_x = tar.position.x;
    target_y = tar.position.y;
    target_z = tar.position.z;
}
int main(int argc, char **argv)
{
    //初始化节点
	ros::init(argc, argv, "moveit_cartesian_demo");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/tracker/tar_pose", 1000, TarCB);
    //引入多线程
	ros::AsyncSpinner spinner(1);
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
    move_group.setGoalOrientationTolerance(0.02);
 
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

    sleep(3);  //停1s
 

    ROS_INFO("next move");


// geometry_msgs::Pose target_pose; // 设置目标姿势

// // 设置目标姿势的位置和朝向

// move_group.setPoseTarget(target_pose);


    // std::vector<double> joint_values ={-1.574390172958374, 1.5622552633285522, -1.565582036972046, 1.57, 1.5742380619049072, -0.7827801704406738} ;
    // move_group.setJointValueTarget(joint_values); // 设置目标关节角度



    // // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    // {
    //     move_group.move();
    //     ROS_INFO("success");
    // }
    // else{ROS_ERROR("fail");}
    // sleep(3);
    geometry_msgs::Pose  curr_pose;
    // 获取当前位姿数据最为机械臂运动的起始位姿
    geometry_msgs::Pose start_pose = move_group.getCurrentPose(end_effector_link).pose;
    
    // float joint_bias = 0.02;
    // if(abs(start_pose.position.x-1.574390172958374)<joint_bias && )

    // 创建Eigen向量表示中心点的位置
    Eigen::Vector3d target_position(target_x-0.5, target_y-0.5, target_z);

    std::cout << "start_pose:" << start_pose.position.x << ", "  << start_pose.position.y << ", " << start_pose.position.z << ", " 
     << start_pose.orientation.w << ", "  << start_pose.orientation.x << ", " << start_pose.orientation.y << ", " << start_pose.orientation.z
     <<std::endl;

    // start_pose.position.x += 0.3;

    // move_group.setPoseTarget(start_pose);

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    // {
    //     move_group.move();
    //     sleep(30);
    //     ROS_INFO("success");
    // }
    // else{ROS_ERROR("fail");}

    // curr_pose=move_group.getCurrentPose(end_effector_link).pose;

    // std::cout << "start_pose1:" << curr_pose.position.x << ", "  << curr_pose.position.y << ", " << curr_pose.position.z << ", " 
    //  << curr_pose.orientation.w << ", "  << curr_pose.orientation.x << ", " << curr_pose.orientation.y << ", " << curr_pose.orientation.z
    //  <<std::endl;

    //初始化路径点向量
	std::vector<geometry_msgs::Pose> waypoints;
    //将初始位姿加入路点列表
	waypoints.push_back(start_pose);    

    float r = 0.1;
    int points_num =20;
    geometry_msgs::Pose tmp;
    geometry_msgs::Pose current_pose;
    Eigen::Quaterniond desired_orientation;  
    Eigen::Vector3d reference_vector(0.0, 0.0, -1.0);  // 参考向量，末端应该指向该向量的方向 
    desired_orientation.setFromTwoVectors(reference_vector, reference_vector); 
    tmp=start_pose;
    // tmp.orientation=desired_orientation;
    tmp.orientation.w = desired_orientation.w();
    tmp.orientation.x = desired_orientation.x();
    tmp.orientation.y = desired_orientation.y();
    tmp.orientation.z = desired_orientation.z();
    // tmp.position.x+=-0.1;
    // tmp.position.z+=-0.2;
    waypoints.push_back(tmp);

    for(int i = 0 ; i < points_num; ++i){
        tmp=start_pose;

        current_pose =  move_group.getCurrentPose(end_effector_link).pose;

        tmp.position.x+=r * cos(2*PI/points_num*i);
        tmp.position.y+=r * sin(2*PI/points_num*i);
        // tmp.position.x+=-0.1;
        // tmp.position.z+=-0.2;


        Eigen::Vector3d current_position(tmp.position.x, tmp.position.y, tmp.position.z);
        // 计算机械臂的面朝方向向量
        Eigen::Vector3d direction_vector = (target_position - current_position).normalized();
        // Eigen::Vector3d direction_vector(0,0,-1);
        // Eigen::Vector3d direction_vector(0.330367,0.6674,-0.667409);
        std::cout << "d_vector:" << direction_vector[0] <<", "<< direction_vector[1]  <<", "<< direction_vector[2] << std::endl;
        // 计算朝向的四元数

        desired_orientation.setFromTwoVectors(reference_vector, direction_vector);
        // // 将四元数与机械臂的当前姿势合并
        // Eigen::Quaterniond current_orientation(
        //     start_pose.orientation.w, start_pose.orientation.x,
        //     start_pose.orientation.y, start_pose.orientation.z);

        Eigen::Quaterniond final_orientation =  desired_orientation;

        tmp.orientation.w = final_orientation.w();
        tmp.orientation.x = final_orientation.x();
        tmp.orientation.y = final_orientation.y();
        tmp.orientation.z = final_orientation.z();


        waypoints.push_back(tmp);
        // move_group.setPoseTarget(tmp);

        // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if (success)
        // {
        //     move_group.move();
        //     sleep(3);
        //     ROS_INFO("success");
        // }
        // else{ROS_ERROR("fail");}

    }
	
    // // start_pose.position.z -= 0.2;
	// // waypoints.push_back(start_pose);
 
    // // start_pose.position.x += 0.1;
	// // waypoints.push_back(start_pose);
 
    // // start_pose.position.y += 0.1;
	// // waypoints.push_back(start_pose);
 
	// 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
 
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the move_group.");
 
	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;
 
	    // 执行运动
	    move_group.execute(plan);
        sleep(10);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    sleep(1);
    // 控制机械臂先回到初始化位置
    // move_group.setNamedTarget("home");
    // move_group.move();
    // sleep(1);
 
	ros::shutdown(); 
	return 0;
}