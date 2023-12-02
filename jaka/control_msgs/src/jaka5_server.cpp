#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "actionlib/server/action_server.h"        
#include "actionlib/server/server_goal_handle.h" 
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/RobotTrajectory.h>
#include<control_msgs/MovejCallbackService.h>



#include "robot_msgs/Move.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "robot_msgs/RobotMsg.h"
#include "robot_msgs/SetUserFrame.h"
#include "robot_msgs/SetTcp.h"
#include "robot_msgs/SetLoad.h"
#include "robot_msgs/ServoL.h"
#include "robot_msgs/ClearErr.h"
#include "robot_msgs/SetCollision.h"
#include "robot_msgs/SetAxis.h"


// #include <learn_service/multinum.h>
int g_count=0;
bool g_cout_failure=false;

class FollowJointTrajectoryAction1
{
  protected:
    
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    moveit_msgs::RobotTrajectory moveit_tra;
    //  control_msgs::FollowJointTrajectoryActionGoal goal_;
    control_msgs::FollowJointTrajectoryActionFeedback feedback_;
    //control_msgs::FollowJointTrajectoryResult result_;  
    //定义action服务端
    //actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>  as_;    

    //定义action服务端目标控制句柄
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_;

    //用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了
    control_msgs::FollowJointTrajectoryResult result_;         
  
   public:
    FollowJointTrajectoryAction1();
    //  FollowJointTrajectoryAction::FollowJointTrajectoryAction():
 
    void goalCB(const actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalConstPtr& goal);

     ~FollowJointTrajectoryAction1(void)
     {
     }
    private:
    ros::ServiceClient add_turtle;

 };

   FollowJointTrajectoryAction1::FollowJointTrajectoryAction1() :
       as_(nh_, "jaka5/follow_joint_trajectory", boost::bind(&FollowJointTrajectoryAction1::goalCB,this,_1), false)
      //  action_name_("jaka5/follow_joint_trajectory")
     {
      std::cout<<"as_.start"<<std::endl;
       as_.start();//action服务端启动
        std::cout<<"as_.start!!!!"<<std::endl;
     }

// const actionlib_tutorials::FibonacciGoalConstPtr &goal
void FollowJointTrajectoryAction1::goalCB(const actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalConstPtr& goal)
{

      /**********************************************************************************************************************************************/
    // std::cout<<"jaka_server!!"<<std::endl;
    int n_joints = goal->trajectory.joint_names.size();
    int n_tra_Points = goal->trajectory.points.size();
    // moveit_tra.joint_trajectory.header.frame_id = goal->trajectory.header.frame_id;
    // moveit_tra.joint_trajectory.joint_names = goal->trajectory.joint_names;
    // moveit_tra.joint_trajectory.points.resize(n_tra_Points);

    // for(int i=0; i<n_tra_Points; i++) // 遍历每组路点
    // {
    //     moveit_tra.joint_trajectory.points[i].positions.resize(n_joints);
    //     moveit_tra.joint_trajectory.points[i].velocities.resize(n_joints);
    //     moveit_tra.joint_trajectory.points[i].accelerations.resize(n_joints);

    //     moveit_tra.joint_trajectory.points[i].time_from_start = goal->trajectory.points[i].time_from_start;
    //     for(int j=0;j<n_joints; j++) // 遍历每组路点中的每个关节数据
    //     {
    //         moveit_tra.joint_trajectory.points[i].positions[j] = goal->trajectory.points[i].positions[j];
    //         moveit_tra.joint_trajectory.points[i].velocities[j] = goal->trajectory.points[i].velocities[j];
    //         moveit_tra.joint_trajectory.points[i].accelerations[j] =goal->trajectory.points[i].accelerations[j];
    //     }
    // }

    // std::cout << "The trajectory data is:" << "********************************************" << std::endl;
    // std::cout << moveit_tra;
    // std::cout<<goal;
    // std::cout << "********************************************" << "The trajectory data is finished printing." << std::endl;

    ROS_INFO("The number of joints is %d.",n_joints);
    ROS_INFO("The waypoints number of the trajectory is %d.",n_tra_Points);
    as_.setSucceeded();
  
    // goal_handle_->setSucceeded();

    // success making  in service

    // moveit_tra.joint_trajectory.header.frame_id = goal->trajectory.header.frame_id;
    // moveit_tra.joint_trajectory.joint_names = goal->trajectory.joint_names;
    // moveit_tra.joint_trajectory.points.resize();
    // srv.request.trajectory.joint_trajectory.header.frame_id= goal->trajectory.header.frame_id;
    // srv.request.trajectory.joint_trajectory.joint_names=goal->trajectory.joint_names;
    // srv.request.trajectory.joint_trajectory.points.resize(n_tra_Points);
    // ros::NodeHandle n;
    ros::service::waitForService("/move_joint");
    std::cout<<"ros::service::waitForService"<<std::endl;
    add_turtle = nh_.serviceClient<control_msgs::MovejCallbackService>("/move_joint");
    control_msgs::MovejCallbackService srv;
    std::cout<<"as_.start!!!!"<<std::endl;
 
    std::cout<<"as_.start!!!!"<<std::endl;  
    // srv.request.mvvelo = 0.6;
    // srv.request.mvacc  = 0.09; 
    srv.request.mvvelo = 0.1;
    srv.request.mvacc  = 0.04;
    for(int i=0; i<n_tra_Points; i++) // 遍历每组路点
    {  
        srv.request.pose.clear();
        srv.request.pose.push_back(goal->trajectory.points[i].positions[0]);
        srv.request.pose.push_back(goal->trajectory.points[i].positions[1]);
        srv.request.pose.push_back(goal->trajectory.points[i].positions[2]);
        srv.request.pose.push_back(goal->trajectory.points[i].positions[3]);
        srv.request.pose.push_back(goal->trajectory.points[i].positions[4]);
        srv.request.pose.push_back(goal->trajectory.points[i].positions[5]);
        
        // if(add_turtle.call(srv))
        // {
        //     ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        //     ROS_INFO("Response from server ret: %d",srv.response.ret);
        // }else{
        //     ROS_ERROR("failed to call /robot_driver/move_line");
        //     // return 1;
        // }
        
    }

    if(add_turtle.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);
    }else{
        ROS_ERROR("failed to call /robot_driver/move_line");
        // return 1;
    }

}
/****************************************************************************************************************************************************/


  int main(int argc, char** argv)
   {
    ros::init(argc, argv, "jaka5_server");
    std::cout<<"jaka_server"<<std::endl;
        // 创建节点句柄
	  ros::NodeHandle node;
  // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service

    //  ros::ServiceClient service_client_=nh_.serviceClient<control_msgs::MovejCallbackService>("move_joint");
    //   control_msgs::MovejCallbackService srv;   
    FollowJointTrajectoryAction1 followjointtrajectoryaction1;
    // followjointtrajectoryaction1.~FollowJointTrajectoryAction1();
    ros::spin();
   
     return 0;
   }





  //  int main(int argc, char** argv)
  //  {
  //    ros::init(argc, argv, "followjoint");
   
  //    FollowJointTrajectoryAction followjoint("followjoint");
  //    ros::spin();
   
  //    return 0;
  //  }