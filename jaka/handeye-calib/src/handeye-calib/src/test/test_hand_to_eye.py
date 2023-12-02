#!/usr/bin/env python3
# coding: utf-8
"""
手眼标定测试程序，眼在手上
输入：
1.aruco码在相机坐标系下的坐标 camera_frame->aruco_marker_frame
2.手眼标定结果base_link->camera_frame
输出：
1.aruco码在机械臂基坐标下的位置,base_link->aruco_maker_frame

单独测试代码：
1.rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link link7_name
2.rosrun tf2_ros static_transform_publisher 0 0 2 0 0 0 1 camera_frame aruco_marker_frame
3.roslaunch handeye-calib test_hand_to_eye_calib.launch

"""
import rospy
import tf
import transforms3d as tfs
import geometry_msgs.msg
import sys
import numpy as np
import math
from tf2_msgs.msg import TFMessage
import json
  

if __name__ == '__main__':
    rospy.init_node('test_hand_to_eye')   
    base_link = rospy.get_param("/test_hand_to_eye/base_link")
    end_link = rospy.get_param("/test_hand_to_eye/end_link")
    camera_link = rospy.get_param("/test_hand_to_eye/camera_link")
    marker_link = rospy.get_param("/test_hand_to_eye/marker_link")

    base_link2camera_link = rospy.get_param("/test_hand_to_eye/base_link2camera_link")
    base_link2camera_link = json.loads(base_link2camera_link.replace("'",'"'))

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()  

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans2,rot2) = base_link2camera_link['t'], base_link2camera_link['r']
            br.sendTransform(trans2,rot2,rospy.Time.now(),camera_link,base_link)
            # print("publish:%s->%s, %s,%s" % (base_link,camera_link,trans2,rot2))

            (trans1,rot1) = listener.lookupTransform(base_link,marker_link, rospy.Time(0))
            print("result:%s->%s, %s,%s" % (base_link,marker_link,trans1,rot1))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()