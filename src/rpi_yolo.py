#!/usr/bin/env python3
from __future__ import print_function
import rospy
from sensor_msgs.msg import CameraInfo, Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from pytorchYolo.argLoader import ArgLoader
from pytorchYolo.detector import YoloLiveVideoStream
from pytorchYolo import utils
from pytorchYolo import constants
import yaml
import torch
from torch.autograd import Variable



class YoloRosWrapper:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("raspicam_node/image",Image,self.img_callback)
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
        # not sure what the loop rate does. higher value seems to give faster framerate
        # but might be costing more to the desktop
        loop_rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            (rows,cols,channels) = self.cv_image.shape
            if cols > 60 and rows > 60:
                # code is copied form img_stream function in the 
                # YoloLiveVideoStream class
                img = self.cv_image
                orig_im = img
                img = utils.prepare_image(img, (detector._img_size,detector._img_size))
                im_dim = torch.FloatTensor((orig_im.shape[1], orig_im.shape[0])).repeat(1,2)
                if detector.gpu:
                  im_dim = im_dim.cuda()
                  img = img.cuda()

                with torch.no_grad():
                  prediction = detector.model.forward(Variable(img), detector.gpu)
                prediction = utils.filter_transform_predictions(prediction, detector.num_classes, detector._conf_thresh, detector._nms_thresh)

                # now have the prediction tensor loaded, use to get data for publishing
                try:
                  num_predictions = prediction.size(0)
                  for pred_index in range(num_predictions):
                    bb_top_left_x = prediction[pred_index][1]
                    bb_top_left_y = prediction[pred_index][2]
                    bb_bot_right_x = prediction[pred_index][3]
                    bb_bot_right_y = prediction[pred_index][4]
                    pred_class = prediction[pred_index][7]



                except AttributeError:
                  # no predicitions -> utils returns 0, int
                  pass



            loop_rate.sleep()

if __name__ == "__main__":
    rospy.init_node("ros_YOLO_wrapper")
    yoloRosWrapper = YoloRosWrapper()
    yoloRosWrapper.run()

