#include <ros/ros.h>
#include "robot_msgs/Move.h"
#include <cstdlib>
#include <vector>
class jaka_arm{
    public:

    jaka_arm(ros::NodeHandle &_nh):
    nh(_nh)
{
        // nh=_nh;
        init_pose();
        flush_srv();

        client_joint = nh.serviceClient<robot_msgs::Move>("/robot_driver/move_joint");
        client_line = nh.serviceClient<robot_msgs::Move>("/robot_driver/move_line");
    }
    void flush_srv(){
        srv.request.pose = pose;
        srv.request.has_ref= 0;
        std::vector<float> ref_joint(0,1);
        srv.request.ref_joint=ref_joint;
        srv.request.mvvelo=2.0;
        srv.request.mvacc=0.5;
        srv.request.mvtime=0.0;
        srv.request.mvradii=0.0;
        srv.request.coord_mode=0;
        srv.request.index=0;
    }
    void init_pose(){// joint
        set_pose(1.6054138696075382,1.5531737196619573,1.520179300016476,0.058488419009407225,-1.5090622400528064,-1.8175936680658081);
    }
    std::vector<float>  set_pose(float v0, float v1,float v2,float v3,float v4,float v5){
        // std::vector<float> pose ={v0, v1, v2 , v3 ,v4 , v5} ;
        pose[0]=v0;pose[1]=v1;pose[2]=v2;pose[3]=v3;pose[4]=v4;pose[5]=v5;
        return pose;
    }
    void run(bool if_joint = true){
        std::cout << pose[0] << pose[1] << pose[2] << pose[3] << pose[4] << pose[5] <<std::endl;
        if (client_joint.call(srv))
        {
            std::cout << "1  "<<srv.response.ret<< " "<<srv.response.message<<std::endl;
            ROS_INFO("id: %d, msg: %s", (int)srv.response.ret, srv.response.message);
        }
        else
        {
            std::cout << "2"<<std::endl;
            ROS_ERROR("Failed to call service add_two_ints");
            // return 1;
        }
    }
    private:
    robot_msgs::Move srv;
    std::vector<float> pose={0,0,0,0,0,0};
    // ros related
    
    ros::NodeHandle &nh;
    ros::ServiceClient client_joint;
    ros::ServiceClient client_line;
    public:

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "jaka_control");
//   if (argc != 3)
//   {
//     ROS_INFO("usage: add_two_ints_client X Y");
//     return 1;
//   }

  ros::NodeHandle n;
  
  jaka_arm robot(n);
  robot.set_pose(1.6054138696075382,1.5531737196619573,1.520179300016476,0.058488419009407225,-1.5090622400528064,-0.8175936680658081);
  robot.run();


  return 0;
}