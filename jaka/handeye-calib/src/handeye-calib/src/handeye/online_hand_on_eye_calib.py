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


real_online_pose = None
real_camera_pose = None


def online_callback(pose):
    global real_online_pose
    # rospy.loginfo(pose.pose)
    real_online_pose = pose.pose


def camera_callback(pose):
    global real_camera_pose
    real_camera_pose = pose.pose


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
    result_path = "/tmp/online_handeye_result_"+str(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))+".txt"
    if result_path is not None:
        file_operate.save_file(result_path,save_data)
        rospy.loginfo("Save result to  "+str(result_path))


if __name__ == '__main__':
    rospy.init_node("online_hand_on_eye_calib", anonymous=False)
    hand_calib = HandeyeCalibrationBackendOpenCV()
    samples = []
    online_pose_topic = rospy.get_param(
        "/online_hand_on_eye_calib/arm_pose_topic")
    camera_pose_topic = rospy.get_param(
        "/online_hand_on_eye_calib/camera_pose_topic")
    rospy.loginfo("手眼标定需要两个位置和姿态，一个是机械臂末端的位姿，将从话题%s中获取 ,另一个相机中标定版的位置姿态将从话题%s获取，所以请确保两个话题有数据" % (str(online_pose_topic),str(camera_pose_topic)))
    rospy.Subscriber(online_pose_topic, PoseStamped, online_callback,queue_size=10)
    rospy.Subscriber(camera_pose_topic, PoseStamped, camera_callback,queue_size=10)

    while not rospy.is_shutdown():
        time.sleep(1)
        if real_online_pose == None:
            rospy.loginfo('等待机械臂位置和姿态话题到位 ...')
        elif real_camera_pose == None:
            rospy.loginfo('等待相机姿态话题数据到位 ...')
        else:
            break


    while not rospy.is_shutdown():
        command = str(input("指令: r 记录,c 计算,s  保存,q  退出:"))

        if command == "r":
            samples.append( {"robot": real_online_pose, "optical": real_camera_pose})
            print("当前已有数据组数:"+str(len(samples)))
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
