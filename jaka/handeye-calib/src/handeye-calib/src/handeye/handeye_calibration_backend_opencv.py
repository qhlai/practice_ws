#!/usr/bin/env python3
# coding: utf-8
import sys
import numpy as np
import transforms3d as tfs
from rospy import logerr, logwarn, loginfo
import math
import sys,os

if os.path.exists('/opt/ros/kinetic/lib/python2.7/dist-packages'):
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2
    sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
elif  os.path.exists('/opt/ros/melodic/lib/python2.7/dist-packages'):
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
    import cv2
    sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
else:
    import cv2



class HandeyeCalibrationBackendOpenCV(object):
    MIN_SAMPLES = 2  # TODO: correct? this is what is stated in the paper, but sounds strange
    """Minimum samples required for a successful calibration."""


    def __init__(self) -> None:
        super().__init__()
        print("======================================================")
        # print("          欢迎使用手眼标定程序，我是小鱼   ")
        # print("        学习机器人一定要关注公众号鱼香ROS      ")
        # print("         回复手眼标定可获得算法推导及实现      ")
        print("------------------------------------------------------")
        print("""            本程序中相关坐标系定义解析                 """)
        print("""   base_link:机械臂基坐标系（一般在固定在机器人底座）         """)
        print("""   end_link :机械臂末端坐标系（可在launch中配置）         """)
        print("""   camera   :相机坐标系         """)
        print("""   marker   :标定版所在坐标系         """)
        print("======================================================")

    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
       # 'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    @staticmethod
    def _msg_to_opencv(transform_msg):
        """
        小鱼：将ROS消息转换成opencv格式
        """
        if isinstance(transform_msg, dict):
            cmt = transform_msg["position"]
            tr = np.array((cmt['x'], cmt['y'], cmt['z']))
            cmq = transform_msg["orientation"]
            rot = tfs.quaternions.quat2mat(
                (cmq['w'], cmq['x'], cmq['y'], cmq['z']))
            return rot, tr
        else:
            cmt = transform_msg.position
            tr = np.array((cmt.x, cmt.y, cmt.z))
            cmq = transform_msg.orientation
            rot = tfs.quaternions.quat2mat((cmq.w, cmq.x, cmq.y, cmq.z))
            return rot, tr


    @staticmethod
    def _test_data(data):
        """
        测试数据，求出平均值和方差，标准差
        """
        mean_result = ['mean']
        var_result = ['var']
        std_result = ['std']
        row_len = len(data)
        column_len = len(data[0])
        for i in range(column_len):
            temp = []
            for j in range(row_len):
                temp.append(data[j][i])
            if  isinstance(temp[0],float):
                mean_result.append(np.mean(temp))
                var_result.append(np.var(temp))
                std_result.append(np.std(temp))
        return mean_result,var_result,std_result

    @staticmethod
    def _get_opencv_samples(samples,eye_on_hand=True):
        """
        Returns the sample list as a rotation matrix and a translation vector.
        :rtype: (np.array, np.array)
        """
        hand_base_rot = []
        hand_base_tr = []
        marker_camera_rot = []
        marker_camera_tr = []

        for s in samples:
            camera_marker_msg = s['optical']
            (mcr, mct) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(
                camera_marker_msg)
            # 眼在手外，手眼调换
            if not eye_on_hand:
                temp = tfs.affines.compose(np.squeeze(mct), mcr, [1, 1, 1])
                temp = np.linalg.inv(temp)
                mct = temp[0:3,3:4]
                mcr = temp[0:3,0:3]

            marker_camera_rot.append(mcr)
            marker_camera_tr.append(mct)

            base_hand_msg = s['robot']
            (hbr, hbt) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(base_hand_msg)
            hand_base_rot.append(hbr)
            hand_base_tr.append(hbt)

        return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)

    @staticmethod
    def _distance(x,y,z):
        return math.sqrt(x*x+y*y+z*z)


    def compute_calibration(self, samples, algorithm=None,eye_on_hand=True):
        """
        Computes the calibration through the OpenCV library and returns it.

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        if algorithm is None:
            algorithm = 'Tsai-Lenz'

        if len(samples) < HandeyeCalibrationBackendOpenCV.MIN_SAMPLES:
            logwarn("{} more samples needed! Not computing the calibration".format(
                HandeyeCalibrationBackendOpenCV.MIN_SAMPLES - len(samples)))
            return

        # Update data
        opencv_samples = HandeyeCalibrationBackendOpenCV._get_opencv_samples(samples,eye_on_hand)
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = opencv_samples

        if len(hand_world_rot) != len(marker_camera_rot):
            logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        method = HandeyeCalibrationBackendOpenCV.AVAILABLE_ALGORITHMS[algorithm]
        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(hand_world_rot, hand_world_tr, marker_camera_rot,
                                                               marker_camera_tr, method=method)

        # end_link->marker or end_link->camera
        result = tfs.affines.compose(np.squeeze(hand_camera_tr), hand_camera_rot, [1, 1, 1])
        (rx, ry, rz) = [math.degrees(float(i)) for i in tfs.euler.mat2euler(hand_camera_rot)]
        (qx, qy, qz , qw) = [float(i) for i in tfs.quaternions.mat2quat(hand_camera_rot)]
        # (qx, qy, qz qw) = [float(i) for i in tfs.euler.euler2quat(rx,ry,rz)]
        (hctx, hcty, hctz) = [float(i) for i in hand_camera_tr]
        
        final_pose = [[algorithm,'x','y','z','rx','ry','rz',"四元数姿态(w,x,y,z)","距离"]]
        for i in range(len(samples)):
            # base_link->end_link
            pose1 = tfs.affines.compose(np.squeeze(hand_world_tr[i]), hand_world_rot[i], [1, 1, 1])
            # marker->camera  or camera->marker
            pose2 = tfs.affines.compose(np.squeeze(marker_camera_tr[i]), marker_camera_rot[i], [1, 1, 1])
            # base_link->end_link->marker   or base_link->end_link->camera 
            temp = np.dot(pose1,result)
            # base_link->end_link->marker->camera  or base_link->end_link->camera->marker
            temp = np.dot(temp,pose2)
            tr = temp[0:3,3:4].T[0]
            rot =tfs.euler.mat2euler(temp[0:3,0:3]) 
            quat = tfs.quaternions.mat2quat(temp[0:3,0:3])
            # quat = [float(i) for i in tfs.euler.euler2quat(rot[0],rot[1],rot[2])]

            if eye_on_hand:
                final_pose.append(["base_link->marker"+str(i),tr[0],tr[1],tr[2],rot[0],rot[1],rot[2],[quat[0],quat[1],quat[2],quat[3]],HandeyeCalibrationBackendOpenCV._distance(tr[0],tr[1],tr[2])])
            else:
                final_pose.append(["base_link->camera"+str(i),tr[0],tr[1],tr[2],rot[0],rot[1],rot[2],[quat[0],quat[1],quat[2],quat[3]],HandeyeCalibrationBackendOpenCV._distance(tr[0],tr[1],tr[2])])
                

        test_result = HandeyeCalibrationBackendOpenCV._test_data(final_pose[1:])
        final_pose.append(test_result[0])
        final_pose.append(test_result[1])
        final_pose.append(test_result[2])

        result_tuple = (hctx, hcty, hctz,rx,ry,rz,[qx, qy, qz , qw]),final_pose
        return result_tuple
