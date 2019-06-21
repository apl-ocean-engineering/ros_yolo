#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo, Image
from pytorchYolo import argLoader, detector


if __name__ == "__main__":
    name = "info_pub"
    rospy.init_node(name)
    argloader = argLoader.ArgLoader()
    print(argloader.args)
    
    