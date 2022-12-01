#!/usr/bin/env python3
from json import JSONDecodeError
from typing import Union

import rospy
import sys
import cv2
import numpy as np
import tf
from d3_apriltag.msg import AprilTagDetection, AprilTagDetectionArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped, Point, Quaternion
from nav_msgs.msg import Odometry
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from apriltag import apriltag
from scipy.spatial.transform import Rotation as R
import json


class apriltag_odom:
    def __init__(self, num_frames: int):
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber('/imx390/image_raw_rgb', Image, self.callback)
        # self.tag_pub = rospy.Publisher('/apriltag_detections', AprilTagDetectionArray, queue_size=10)
        # self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.tfbr = tf.TransformBroadcaster()
        tfl = tf.TransformListener()
        t1 = "map"
        t2 = "apriltag21"
        rospy.sleep(1)
        now = rospy.Time.now()
        tfl.waitForTransform(t1, t2, now, rospy.Duration(4.0))
        self.tag_height = tfl.lookupTransform(t1, t2, now)[0][2]
        self.num_frames = num_frames

    def averageQuaternions(self, Q):
        # Number of quaternions to average
        M = Q.shape[0]
        A = np.zeros((4, 4))

        for i in range(0, M):
            q = Q[i, :]
            # multiply q with its transposed version q' and add A
            A = np.outer(q, q) + A

        # scale
        A = (1.0 / M) * A
        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = np.linalg.eig(A)
        # Sort by largest eigenvalue
        eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]
        # return the real part of the largest eigenvector (has only real part)
        return np.real(eigenVectors[:, 0].flatten())

    def get_instant_pose(self, image: Image) -> Union[Pose, None]:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
            detector = apriltag("tagStandard41h12")
            detections = detector.detect(cv_image)
            data = dict()
            logline = str(len(detections)) + " AprilTags detected.\n"
            tag_detections = []
            for d in detections:
                corn = np.array(d['lb-rb-rt-lt'], dtype=np.float32)
                logline += "ID: " + str(d['id']) + ", LB: " + str(np.int32(corn[0])) + ", RB: " + str(
                    np.int32(corn[1])) + ", RT: " + str(np.int32(corn[2])) + ", LT: " + str(np.int32(corn[3])) + "\n"
                # Corner Refinement
                winsize = (5, 5)
                zerozone = (-1, -1)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
                corn_ref = corn
                # corn_ref = cv2.cornerSubPix(cv_image, corn, winsize, zerozone, criteria)
                logline += "[Refined] ID: " + str(d['id']) + ", LB: " + str(np.int32(corn_ref[0])) + ", RB: " + str(
                    np.int32(corn_ref[1])) + ", RT: " + str(np.int32(corn_ref[2])) + ", LT: " + str(
                    np.int32(corn_ref[3])) + "\n"

                global camera_info
                tag_corners = np.array([corn_ref[0], corn_ref[1], corn_ref[3], corn_ref[2]], dtype=np.float32)
                tag_shape = (2, 2)
                tag_size = 0.3  # m
                # tag_size = 0.1042
                # Define target points. BL, BR, TL, TR
                objp = np.zeros((tag_shape[0] * tag_shape[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:tag_shape[0], 0:tag_shape[1]].T.reshape(-1, 2)
                objp = objp * tag_size

                # Find the rotation and translation vectors.
                iterations = 100  # Default 100
                reproj_error = 1.0  # Default 8.0
                confidence = 0.99  # Default 0.99
                pnp_flag = cv2.SOLVEPNP_ITERATIVE
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                _, rvecs, tvecs, inliers = cv2.solvePnPRansac( \
                    objp, tag_corners, camera_info, None, iterationsCount=iterations, reprojectionError=reproj_error,
                    confidence=confidence, flags=pnp_flag)

                refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, sys.float_info.epsilon)
                rvecs, tvecs = cv2.solvePnPRefineLM(objp, tag_corners, camera_info, None, rvecs, tvecs, refine_criteria)
                rospy.loginfo(tvecs)

                # Create a AprilTagDetection message
                tag_pose_quat = R.from_rotvec(rvecs.squeeze()).as_quat()
                tag_pose = Pose(Point(float(tvecs[0]), float(tvecs[1]), float(tvecs[2])),
                                Quaternion(float(tag_pose_quat[0]), float(tag_pose_quat[1]), float(tag_pose_quat[2]),
                                           float(tag_pose_quat[3])))
                tag_pose_cov = PoseWithCovariance(tag_pose, [0] * 36)
                tag_pose_header = std_msgs.msg.Header()
                tag_pose_header.stamp = rospy.Time.now()
                tag_pose_cov_stamp = PoseWithCovarianceStamped(tag_pose_header, tag_pose_cov)
                tag_detection = AprilTagDetection([d['id']], [tag_size], tag_pose_cov_stamp, np.array(
                    [corn_ref[0][0], corn_ref[0][1], corn_ref[1][0], corn_ref[1][1], corn_ref[2][0], corn_ref[2][1],
                     corn_ref[3][0], corn_ref[3][1]], dtype=np.int32))
                tag_detections.append(tag_detection)

                # Invert the Tag Pose to get the Robot's Pose
                rotmat = R.from_rotvec(rvecs.squeeze()).as_matrix()
                tfmat = np.hstack((rotmat, tvecs))
                tfmat = np.vstack((tfmat, [0, 0, 0, 1]))
                print("tfmat:")
                print(tfmat)
                res_tfmat = np.linalg.inv(tfmat)
                print("tfmat linalg:")
                print(res_tfmat)
                res_tvecs = res_tfmat[:3, 3]
                res_rotmat = res_tfmat[:3, :3]
                print("res_tvecs:")
                print(res_tvecs)
                print("res_rotmat:")
                print(res_rotmat)

                # Create a Robot Pose
                robot_pose_quat = R.from_matrix(res_rotmat).as_quat()
                robot_pose = Pose(Point(float(res_tvecs[0]), float(res_tvecs[1]), float(res_tvecs[2])),
                                  Quaternion(float(robot_pose_quat[0]), float(robot_pose_quat[1]),
                                             float(robot_pose_quat[2]), float(robot_pose_quat[3])))

                # Create an Odometry message for the robot
                # odom = Odometry()
                # odom.header.stamp = rospy.Time.now()
                # odom.header.frame_id = "apriltag_"+str(d['id'])
                # odom.pose.pose = tag_pose
                # odom.pose.pose = robot_pose
                # self.odom_pub.publish(odom)
                rospy.loginfo(logline)
                return robot_pose
                # Broadcast Transform
                # self.tfbr.sendTransform((res_tvecs[0], res_tvecs[1], res_tvecs[2]),
                #                         # self.tfbr.sendTransform((res_tvecs[0], res_tvecs[1], 0),
                #                         (
                #                         robot_pose_quat[0], robot_pose_quat[1], robot_pose_quat[2], robot_pose_quat[3]),
                #                         rospy.Time.now(),
                #                         "imx390_rear_optical",
                #                         "apriltag21")
            rospy.loginfo(logline)
            return None
        except CvBridgeError as e:
            rospy.loginfo(e)

    def get_pose(self) -> Pose:
        poses = []
        fails = 0
        while len(poses) < self.num_frames:
            print("Waiting on Frame " + str(len(poses) + 1) + "v" + str(fails) + "/" + str(self.num_frames))
            image = rospy.wait_for_message("/imx390/image_raw_rgb", Image)
            result = self.get_instant_pose(image)
            if result is not None:
                poses.append(result)
                fails = 0
            else:
                fails += 1
        x_sum = 0
        y_sum = 0
        for pose in poses:
            x_sum += pose.position.x
            y_sum += pose.position.y
        x_avg = x_sum / self.num_frames
        y_avg = y_sum / self.num_frames
        poses_to_discard = []
        print("Average X: " + str(x_avg))
        print("Average Y: " + str(y_avg))
        for pose in poses:
            print("Actual X: " + str(pose.position.x))
            print("Actual Y: " + str(pose.position.y))
            print("X Diff: " + str(abs(abs(x_avg) - abs(pose.position.x))))
            print("Y Diff: " + str(abs(abs(y_avg) - abs(pose.position.y))))
            x_bad = abs(abs(x_avg) - abs(pose.position.x)) > 0.1
            y_bad = abs(abs(y_avg) - abs(pose.position.y)) > 0.1
            print("X Bad: " + str(x_bad))
            print("Y Bad: " + str(y_bad))
            if x_bad or y_bad:
                poses_to_discard.append(pose)
        for pose in poses_to_discard:
            poses.remove(pose)
        print("Removed " + str(len(poses_to_discard)) + " poses out of " + str(self.num_frames) + " poses")
        if len(poses) == 0:
            print("All poses were considered bad, trying pose estimation again...")
            return self.get_pose()
        x_sum, y_sum, z_sum = [0] * 3
        conversion_quaternions = []
        for pose in poses:
            x_sum += pose.position.x
            y_sum += pose.position.y
            z_sum += pose.position.z
            conversion_quaternions.append((pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
        plen = len(poses)
        x_pose = float(x_sum / plen)
        y_pose = float(y_sum / plen)
        z_pose = self.tag_height
        avg_point = Point(x_pose, y_pose, z_pose)
        avg_quat_wxyz = self.averageQuaternions(np.array(conversion_quaternions, dtype=float))
        avg_quat_xyzw = Quaternion(avg_quat_wxyz[1], avg_quat_wxyz[2], avg_quat_wxyz[3], avg_quat_wxyz[0])
        print("X Final: " + str(x_pose))
        print("Y Final: " + str(y_pose))
        camera_pose = Pose(avg_point, avg_quat_xyzw)
        self.tfbr.sendTransform((x_pose, y_pose, z_pose),
                                # self.tfbr.sendTransform((res_tvecs[0], res_tvecs[1], 0),
                                (avg_quat_xyzw.x, avg_quat_xyzw.y, avg_quat_xyzw.z, avg_quat_xyzw.w),
                                rospy.Time.now(),
                                "imx390_rear_optical",
                                "apriltag21")
        return camera_pose

def pose_to_dict(pose):
    d = {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z
        },
        "orientation": {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w
        }
    }
    return d

def main(args):
    rospy.init_node('apriltag_detector', anonymous=True)

    # Get camera name from parameter server
    global camera_name
    camera_name = rospy.get_param("~camera_name", "camera")
    camera_info_topic = "/{}/camera_info".format(camera_name)
    rospy.loginfo("Waiting on camera_info: %s" % camera_info_topic)

    # Wait until we have valid calibration data before starting
    global camera_info
    camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)
    camera_info = np.array(camera_info.K, dtype=np.float32).reshape((3, 3))
    #	camera_info = None
    rospy.loginfo("Camera intrinsic matrix: %s" % str(camera_info))
    running = True
    old_command = ""
    num_frames_str = ""
    base_path = "/opt/robotics_sdk/"
    print("AprilTag Pose Estimation")
    while not num_frames_str.isdigit():
        num_frames_str = input("Enter number of frames that should be taken to average per Pose Estimation: ")
    num_frames = int(num_frames_str)
    april = apriltag_odom(num_frames)
    save_file_name = ""
    while running and not rospy.is_shutdown():
        print("AprilTag Pose Estimation Commands: 'getpose', 'savepose', 'exit'")
        prompt_string = ""
        if old_command != "":
            prompt_string = "[" + old_command + "] > "
        else:
            prompt_string = "> "
        command = input(prompt_string)
        if command == "":
            if old_command != "":
                command = old_command
            else:
                continue
        old_command = command
        if command == "getpose":
            april.get_pose()
        elif command == "savepose":
            while save_file_name == "" or save_file_name == base_path:
                save_file_name = base_path + input("Enter name for Pose Save File: ")
            pose_name = ""
            while pose_name == "":
                pose_name = input("Enter name for Saved Pose: ")
            pose = april.get_pose()
            data = dict()
            try:
                new_file = open(save_file_name, "x+")
                print("File doesn't exist, creating...")
                new_file.write("{}")
                new_file.flush()
                new_file.close()
            except FileExistsError:
                print("File exists, loading...")
            with open(save_file_name, "r") as file:
                data = json.load(file)
                print("File loaded!")
            with open(save_file_name, "w") as file:
                data[pose_name] = pose_to_dict(pose)
                json.dump(data, file)
                print("File updated and saved! Path: " + save_file_name)
        elif command == "exit":
            running = False
        else:
            print("Invalid Command")




if __name__ == '__main__':
    print("Starting AprilTag Detector Node")
    main(sys.argv)
