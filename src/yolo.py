#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo, Image
from pytorchYolo import argLoader


if __name__ == "__main__":
    name = "info_pub"
    rospy.init_node(name)
    
    