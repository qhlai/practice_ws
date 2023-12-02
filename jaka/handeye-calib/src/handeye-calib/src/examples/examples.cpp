#include <ros/ros.h>
#include <tf/transform_listener.h>

tf::StampedTransform getTransform(const tf::TransformListener& listener){
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("base_link", "aruco_marker_frame", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("base_link", "aruco_marker_frame", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    return transform;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  tf::TransformListener listener;
  tf::StampedTransform baselink2marker = getTransform(listener);
  return 0;
};