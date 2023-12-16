#!/usr/bin/env python3
# coding:utf-8
"""
将tf数据转换成标定时所需的机械臂话题数据
"""
import rospy
from geometry_msgs.msg import PoseStamped

import tf
import transforms3d as tfs
import geometry_msgs.msg
import numpy as np
import transforms3d as tfs
  



if __name__ == '__main__':
    rospy.init_node('tf_to_pose', anonymous=False)   
    arm_pose_topic = rospy.get_param("/tf_to_pose/arm_pose_topic")
    base_link = rospy.get_param("/tf_to_pose/base_link")
    end_link = rospy.get_param("/tf_to_pose/end_link")

    rospy.loginfo("获取话题和link配置从参数服务器，要发布的话题是%s 从机械臂的%s->%s之间的变换关系" % (str(arm_pose_topic),str(base_link),str(end_link)))

    listener = tf.TransformListener()
    pub = rospy.Publisher(str(arm_pose_topic),PoseStamped,queue_size=10)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            (trans1,rot1) = listener.lookupTransform(str(base_link),str(end_link), rospy.Time(0))
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = trans1[0]
            pose.pose.position.y = trans1[1]
            pose.pose.position.z = trans1[2]

            pose.pose.orientation.x = rot1[0]
            pose.pose.orientation.y = rot1[1]
            pose.pose.orientation.z = rot1[2]
            pose.pose.orientation.w = rot1[3]

            pub.publish(pose)
            # print("base_link->end_link",trans1,rot1)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
