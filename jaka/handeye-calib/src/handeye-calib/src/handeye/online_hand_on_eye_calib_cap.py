#!/usr/bin/env python3
# coding:utf-8
import rospy
import transforms3d as tfs
from geometry_msgs.msg import PoseStamped

from handeye_calibration_backend_opencv import HandeyeCalibrationBackendOpenCV
import math
import time,datetime
import file_operate
from tabulate import tabulate
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

real_online_pose = None
real_camera_pose = None

real_camera_img = None
real_camera_img_depth = None
real_camera_img_aligned = None
calib_status = None

camera_info_Get = None

moveit_pose=None

img_num=0

info_path  = '/home/hanglok/output/'
img_path  = '/home/hanglok/output/rgb/'
img_depth_path  = '/home/hanglok/output/depth/'
img_aligned_path  = '/home/hanglok/output/aligned/'
def online_callback(pose):
    global real_online_pose
    # rospy.loginfo(pose.pose)
    real_online_pose = pose.pose


def camera_callback(pose):
    global real_camera_pose
    real_camera_pose = pose.pose


def camera_img_callback(msg):
    global real_camera_img
    real_camera_img = msg

def camera_img_depth_callback(msg):
    global real_camera_img_depth
    real_camera_img_depth = msg


def moveit_pose_callback(msg):
    global moveit_pose
    moveit_pose = pose.pose

def camera_aligned_callback(msg):
    global real_camera_img_aligned
    real_camera_img_aligned = msg



def camera_info_color_callback(msg):
    pass
    with open(info_path+'camera_info_color_info.txt', 'w') as file:
        file.write(str(msg))
    camera_info_color_sub.unregister()

def camera_info_depth_callback(msg):
    pass
    with open(info_path+'camera_info_depth_info.txt', 'w') as file:
        file.write(str(msg))
    camera_info_depth_sub.unregister()

def camera_info_aligned_callback(msg):
    pass
    with open(info_path+'camera_info_aligned_info.txt', 'w') as file:
        file.write(str(msg))
    camera_info_aligned_sub.unregister()

def calib_cmd_callback(msg):
    pass
    global calib_status
    global real_camera_img
    global real_camera_img_depth
    global bridge
    global img_num
    # if calib_status == None:
    #     calib_status=0
    if calib_status != 1:
        return

    if msg.position[0] ==1:
        samples.append( {"robot": real_online_pose, "optical": real_camera_pose})
        print("当前已有数据组数:"+str(len(samples)))
        cv_img = bridge.imgmsg_to_cv2(real_camera_img, "bgr8")
        image_name = str(img_num)+ ".png" #图像命名：时间戳.jpg
        cv2.imwrite(img_path + image_name, cv_img)  #保存；
        cv_img = bridge.imgmsg_to_cv2(real_camera_img_depth,  desired_encoding='16UC1')
        image_name = str(img_num)+ ".npy" #图像命名：时间戳.jpg
        # cv2.imwrite(img_depth_path + image_name, cv_img)  #保存；
        np_image = np.asarray(cv_img)
        np.save(img_depth_path + image_name, np_image)

        cv_img = bridge.imgmsg_to_cv2(real_camera_img_aligned,  desired_encoding='16UC1')
        image_name = str(img_num)+ ".npy" #图像命名：时间戳.jpg
        np_image = np.asarray(cv_img)
        np.save(img_aligned_path + image_name, np_image)
        
        img_num+=1
        # cv2.imshow("frame" , cv_img)
        # cv2.waitKey(3)
        # cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        if len(samples) > 2:
            temp_sample,final_pose = hand_calib.compute_calibration(samples)
            print(temp_sample)
    elif msg.position[0] ==2:
        pass
        calculate(samples,hand_calib,True)
    elif msg.position[0] ==3:
        pass
        save(calculate(samples,hand_calib,True)+"\n原始数据 :\n\n"+get_csv_from_sample(samples))

    # global real_camera_img_depth
    # real_camera_img_depth = msg
def get_pose_from_ros(pose):
    eulor = tfs.euler.quat2euler(
        (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
    real_pose = [pose.position.x, pose.position.y, pose.position.z,
                 math.degrees(eulor[0]), math.degrees(eulor[1]), math.degrees(eulor[2])]
    return real_pose


def get_csv_from_sample(samples):
    data = ""
    for d in samples:
        data += str("hand,"+str(get_pose_from_ros(d['robot']))[1:-1]+"\n")
        data += str("eye,"+str(get_pose_from_ros(d['optical']))[1:-1]+"\n")
    return data


def calculate(samples,hand_calib,eye_on_hand=True):
    esti_pose = {}
    save_data = ""
    if len(samples) > 2:
        data =  [['算法(end_link->camera)','x','y','z','rx','ry','rz',"四元数姿态(w,x,y,z)","distance"]]
        for algoram in hand_calib.AVAILABLE_ALGORITHMS:
            pose,final_pose = hand_calib.compute_calibration(samples,algorithm=algoram,eye_on_hand=eye_on_hand)
            data.append(["end_link->camera:"+algoram,pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],pose[6],hand_calib._distance(pose[0],pose[1],pose[2])])
            esti_pose[algoram] = final_pose
        print ("\n"+tabulate(data,headers="firstrow") + "\n")
        save_data  += str(  "\n"+tabulate(data,headers="firstrow") + "\n")

        test_result =  hand_calib._test_data(data[1:])
        data = [['算法平均值测试','x','y','z','rx','ry','rz',"距离"]]
        for d in test_result:
            data.append(d)
        print(tabulate(data,headers="firstrow"))
        save_data  += str(  "\n"+tabulate(data,headers="firstrow") + "\n")

        for algoram in hand_calib.AVAILABLE_ALGORITHMS:
            print(tabulate(esti_pose[algoram],headers="firstrow"))
            save_data  += str(  "\n"+tabulate(esti_pose[algoram],headers="firstrow") + "\n")
    else:
        print('数据数量足够')
    return save_data


def save(save_data):
    result_path = info_path+"online_handeye_result_"+str(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))+".txt"
    if result_path is not None:
        file_operate.save_file(result_path,save_data)
        rospy.loginfo("Save result to  "+str(result_path))


if __name__ == '__main__':
    rospy.init_node("online_hand_on_eye_calib", anonymous=False)
    hand_calib = HandeyeCalibrationBackendOpenCV()
    samples = []
    global bridge
    bridge = CvBridge()
    online_pose_topic = rospy.get_param(
        "/online_hand_on_eye_calib/arm_pose_topic")
    camera_pose_topic = rospy.get_param(
        "/online_hand_on_eye_calib/camera_pose_topic")
    camera_img_topic = rospy.get_param(
        "/online_hand_on_eye_calib/cam_rgb")
    camera_img_depth_topic = rospy.get_param(
        "/online_hand_on_eye_calib/cam_depth")
    calib_cmd_topic="/calib_cmd"


    camera_img_aligned_topic="/camera/aligned_depth_to_color/image_raw"

    rospy.loginfo("手眼标定需要两个位置和姿态，一个是机械臂末端的位姿，将从话题%s中获取 ,另一个相机中标定版的位置姿态将从话题%s获取，所以请确保两个话题有数据" % (str(online_pose_topic),str(camera_pose_topic)))
    rospy.Subscriber(online_pose_topic, PoseStamped, online_callback,queue_size=10)
    rospy.Subscriber(camera_pose_topic, PoseStamped, camera_callback,queue_size=10)
    rospy.Subscriber(camera_img_topic, Image, camera_img_callback,queue_size=5)
    rospy.Subscriber(camera_img_depth_topic, Image, camera_img_depth_callback,queue_size=5)
    rospy.Subscriber(camera_img_aligned_topic, Image, camera_aligned_callback,queue_size=5)
    rospy.Subscriber(calib_cmd_topic, JointState, calib_cmd_callback,queue_size=1)
    camera_info_color_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_color_callback)
    camera_info_depth_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, camera_info_depth_callback)
    camera_info_aligned_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, camera_info_aligned_callback)
    while not rospy.is_shutdown():
        time.sleep(1)
        if real_online_pose == None:
            rospy.loginfo('等待机械臂位置和姿态话题到位 ...')
        elif real_camera_pose == None:
            rospy.loginfo('等待相机姿态话题数据到位 ...')
        else:
            break
 

    # while not rospy.is_shutdown():
    #     time.sleep(1)
    #     if real_online_pose == None:
    #         rospy.loginfo('等待机械臂位置和姿态话题到位 ...')
    #     elif real_camera_pose == None:
    #         rospy.loginfo('等待相机姿态话题数据到位 ...')
    #     else:
    #         break
    # global calib_status

    while not rospy.is_shutdown():
        calib_status = 1
        command = str(input("指令: r 记录,c 计算,s  保存,q  退出:"))
        # global calib_status
        

        if command == "r":
            samples.append( {"robot": real_online_pose, "optical": real_camera_pose})
            print("当前已有数据组数:"+str(len(samples)))
            cv_img = bridge.imgmsg_to_cv2(real_camera_img, "bgr8")
            image_name = str(img_num)+ ".jpg" #图像命名：时间戳.jpg
            cv2.imwrite(img_path + image_name, cv_img)  #保存；
            cv_img = bridge.imgmsg_to_cv2(real_camera_img_depth,  desired_encoding='16UC1')
            image_name = str(img_num)+ ".npy" #图像命名：时间戳.jpg
            # cv2.imwrite(img_depth_path + image_name, cv_img)  #保存；
            np_image = np.asarray(cv_img)
            np.save(img_depth_path + image_name, np_image)

            cv_img = bridge.imgmsg_to_cv2(real_camera_img_aligned,  desired_encoding='16UC1')
            image_name = str(img_num)+ ".npy" #图像命名：时间戳.jpg
            np_image = np.asarray(cv_img)
            np.save(img_aligned_path + image_name, np_image)

            img_num+=1
            # cv2.imshow("frame" , cv_img)
            # cv2.waitKey(3)
            # cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
            if len(samples) > 2:
                temp_sample,final_pose = hand_calib.compute_calibration(samples)
                print(temp_sample)

        elif command == 'c':
            calculate(samples,hand_calib,True)

        elif command == 'q':
            break

        elif command == 'p':
            print(get_csv_from_sample(samples))

        elif command == 's':
            save(calculate(samples,hand_calib,True)+"\n原始数据 :\n\n"+get_csv_from_sample(samples))
