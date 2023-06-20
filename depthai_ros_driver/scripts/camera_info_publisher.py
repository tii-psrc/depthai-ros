#!/usr/bin/env python3
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import rospy

l_yaml_fname = '/home/tii/git/luna/kalibr/results/kalibr_calibration/oakd-usb/camera_info_left.yaml'
l_info_topic = "/oak/left/camera_info"
l_image_topic = "/oak/left/image_raw"
r_yaml_fname = '/home/tii/git/luna/kalibr/results/kalibr_calibration/oakd-usb/camera_info_right.yaml'
r_info_topic = "/oak/right/camera_info"
r_image_topic = "/oak/right/image_raw"

def read_cam_info(yaml_fname):
  with open(yaml_fname, "r") as file_handle:
      calib_data = yaml.safe_load(file_handle)
  # Parse
  camera_info_msg = CameraInfo()
  camera_info_msg.width = calib_data["image_width"]
  camera_info_msg.height = calib_data["image_height"]
  camera_info_msg.K = calib_data["camera_matrix"]["data"]
  camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
  camera_info_msg.R = calib_data["rectification_matrix"]["data"]
  camera_info_msg.P = calib_data["projection_matrix"]["data"]
  camera_info_msg.distortion_model = calib_data["distortion_model"]
  return camera_info_msg

l_cam_info = read_cam_info(l_yaml_fname)
r_cam_info = read_cam_info(r_yaml_fname)

#print(read_cam_info(l_yaml_fname))

# Initialize publisher node
rospy.init_node("camera_info_publisher", anonymous=True)
l_publisher = rospy.Publisher(l_info_topic, CameraInfo, queue_size=10)
r_publisher = rospy.Publisher(r_info_topic, CameraInfo, queue_size=10)


def l_image_callback(msg):
    rospy.loginfo_once("left camera_info published")
    l_cam_info.header = msg.header
    l_publisher.publish(l_cam_info)

def r_image_callback(msg):
    rospy.loginfo_once("right camera_info published")
    r_cam_info.header = msg.header
    r_publisher.publish(r_cam_info)

rospy.Subscriber(l_image_topic, Image, l_image_callback)
rospy.Subscriber(r_image_topic, Image, r_image_callback)

rospy.spin()

