#!/usr/bin/env python3
from __future__ import print_function
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image
from ros_yolo.msg import prac
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from pytorchYolo.argLoader import ArgLoader
from pytorchYolo.detector import YoloLiveVideoStream
from manipulation_context_slam_msgs.msg import DetectionBox, DetectionBoxesList


class YoloRosWrapper:
    def __init__(self):
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/left/image_raw",
                Image, self.img_callback)
        self.cv_image = np.zeros((10,10,3), np.uint8)
        self.detection_publisher = rospy.Publisher('/detection/boxes',
                DetectionBoxesList, queue_size = 10)

    def img_callback(self, img):
        try:
          self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
          print(e)

    def load_yolo_detector(self):
          argLoader = ArgLoader()
          args = argLoader.args  # parse the command line arguments
          print(args)
          yoloVideoStream = YoloLiveVideoStream(args)

          return yoloVideoStream

    def run(self):
        detector = self.load_yolo_detector()
        loop_rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            header = Header()
            header.frame_id = '/camera/left'

            (rows,cols,channels) = self.cv_image.shape
            if cols > 60 and rows > 60:
                detection, squares, class_list = detector.stream_img(self.cv_image)
                if detection:

                    db_list = DetectionBoxesList()
                    db_list.header = header
                    for num, sq in enumerate(squares):
                        header.stamp = rospy.Time.now()
                        db = DetectionBox()
                        db.header = header
                        db.id = num
                        db.detection_name = class_list[num]
                        db.lower_x = sq.lower_x
                        db.lower_y = sq.lower_y
                        db.upper_x = sq.upper_x
                        db.upper_y = sq.upper_y

                        db_list.detection_messages.append(db)
                    db_list.detection_nums = num

                    self.detection_publisher.publish(db_list)


            loop_rate.sleep()

if __name__ == "__main__":
    rospy.init_node("ros_YOLO_wrapper")
    yoloRosWrapper = YoloRosWrapper()
    yoloRosWrapper.run()
