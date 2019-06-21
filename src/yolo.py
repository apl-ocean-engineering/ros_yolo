#!/usr/bin/env python3
from __future__ import print_function
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from pytorchYolo.argLoader import ArgLoader
from pytorchYolo.detector import YoloLiveVideoStream


class YoloRosWrapper:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.img_callback)
        self.cv_image = np.zeros((10,10,3), np.uint8)
    def img_callback(self, img):
        try:
          self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
          print(e)
        
    def load_yolo_detector(self):
          argLoader = ArgLoader()
          args = argLoader.args  # parse the command line arguments
          yoloVideoStream = YoloLiveVideoStream(args)
          
          return yoloVideoStream
        
    def run(self):
        detector = self.load_yolo_detector()
        loop_rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            (rows,cols,channels) = self.cv_image.shape
            if cols > 60 and rows > 60:
                detector.stream_img(self.cv_image)
            
            
            loop_rate.sleep()

if __name__ == "__main__":
    rospy.init_node("ros_YOLO_wrapper")
    yoloRosWrapper = YoloRosWrapper()
    yoloRosWrapper.run()    
    