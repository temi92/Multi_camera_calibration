#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo, Image
import message_filters
from cv_bridge import CvBridge
from calibration import MonoCalibration, StereoCalibration
import cv2
import numpy as np
import time
import os
import shutil
import sys


class ImageGrabber(object):
    def __init__(self, image_topics, rows, columns, image_path, delay, no_images, synchronizer = message_filters.ApproximateTimeSynchronizer):
        self.cv_img_left = None
        self.cv_img_right = None
        self.pub = rospy.Publisher("/image_pair", Image, queue_size=10)
        self.bridge = CvBridge()
        images2sync = [(image_topics[0], Image), (image_topics[1], Image)]
        ats = synchronizer([message_filters.Subscriber(topic, type) for (topic, type) in images2sync], slop=0.05,queue_size=5)
        ats.registerCallback(self.callback)
        self.r = rospy.Timer(rospy.Duration(delay), self.save_to_disk)


        self.found_chessboard = [False, False]
        self.found_corners = [False, False]
        self.pattern_size = (rows, columns) 
        self.counter = 0 #counter to keep track of how many images detected.
        self.no_images = no_images #no of images to store to disk
        self.image_path = image_path

    def preprocess_img(self, lmsg, rmsg, scale=1.0):
        #convert img to opencv format and resize.. 
        self.cv_img_left = self.bridge.imgmsg_to_cv2(lmsg, desired_encoding='bgr8')
        self.cv_img_right = self.bridge.imgmsg_to_cv2(rmsg, desired_encoding='bgr8')
        
        #height = img.shape[0]
        #width = img.shape[1]

        if scale < 1.0:
            self.cv_img_left = cv2.resize(self.cv_img_left, (int(self.cv_img_left.shape[1] / scale), int(self.cv_img_left.shape[0]/ scale)))
            self.cv_img_right = cv2.resize(self.cv_img_right, (int(self.cv_img_right.shape[1] / scale), int(self.cv_img_right.shape[0]/ scale)))

    def publish_images(self):
        #draw detected chessboard on images..
        img1, img2 = self.get_chessboard()
        vis = cv2.hconcat([img1, img2])
        self.pub.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))
        rate = rospy.Rate(10)
        rate.sleep()

    def callback(self, lmsg, rmsg):
        self.preprocess_img(lmsg, rmsg)
        self.publish_images()

    def get_chessboard(self):
        tmp_images = [np.copy(self.cv_img_left), np.copy(self.cv_img_right)]
       
        for i, image in enumerate(tmp_images):
            self.found_chessboard[i], self.found_corners[i] = cv2.findChessboardCorners( image, self.pattern_size, flags = cv2.CALIB_CB_FAST_CHECK )
            if self.found_chessboard[i]:
                cv2.drawChessboardCorners(tmp_images[i], self.pattern_size, self.found_corners[i], True )
        return tmp_images

    def save_to_disk(self, event):
        #we make sure chessboard is found in both images before we proceed to saving images
        if all(self.found_chessboard):
        
            self.counter = self.counter + 1
            rospy.logdebug("number of images saved - %d" %self.counter)
            
            number_string = str(self.counter).zfill(len(str(self.no_images)))
            for side, image in zip(("left", "right"), (self.cv_img_left, self.cv_img_right)):
                file_name = "{}_{}.png".format(side, number_string)
                output_path = os.path.join(self.image_path, file_name)
                cv2.imwrite(output_path, image)

        else:
          
            rospy.logwarn("skipping image could not detect checkerboard pattern!")

        if self.counter >= self.no_images:
            rospy.loginfo("finished capturing all the images")
            #shut down thread .
            self.r.shutdown()
        
if __name__ == "__main__":

    
    _node_name = "chessboard_capture"
    rospy.init_node(_node_name, anonymous=True, log_level=rospy.DEBUG)
    
    rows = rospy.get_param("/chessboard_capture/rows")
    columns = rospy.get_param("chessboard_capture/columns")
    square_size = rospy.get_param("chessboard_capture/square-size")
    calibration_images = rospy.get_param("chessboard_capture/calibration_images")
    delay = rospy.get_param("chessboard_capture/delay")
    no_images = rospy.get_param("chessboard_capture/no_images")
    image_topics = (rospy.get_param("chessboard_capture/camera1"), rospy.get_param("chessboard_capture/camera2"))

    try:
        os.mkdir(calibration_images)
    except OSError as error:
        print ("directory already exists")
        sys.exit(1)

    g = ImageGrabber(image_topics, rows, columns, calibration_images, delay, no_images)

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("shutting down..")

  
