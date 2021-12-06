#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena

This file implements a ROS node that subscribes to topics for RGB images,
pointclouds, and camera calibration info, and uses the functions you
implemented to publish a segmented pointcloud to the topic /segmented_points.

Once you are confident in your implementation in image_segmentation.py and
pointcloud_segmentation.py, run this file to begin publishing a segmented
pointcloud.
"""

from __future__ import print_function
from collections import deque

import rospy
import message_filters
import ros_numpy
import tf

### A1 SOCCER ###
import pyrealsense2 as rs2
import cv2
import imutils
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point

from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import numpy as np
import cv2

from cv_bridge import CvBridge

from image_segmentation import *
from pointcloud_segmentation import segment_pointcloud


def get_camera_matrix(camera_info_msg):
    return np.reshape(camera_info_msg.K, (3,3))

### A1 SOCCER ###

def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):
    _intrinsics = rs2.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = rs2.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]
    result = rs2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
    #result[0]: right, result[1]: down, result[2]: forward
    return result[2], -result[0], -result[1]

def isolate_object_of_interest_old(points, image, cam_matrix, trans, rot):
    segmented_image = segment_image(image)
    points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
    return points

def numpy_to_pc2_msg(points):
    return ros_numpy.msgify(PointCloud2, points, stamp=rospy.Time.now(),
        frame_id='camera_depth_optical_frame')

def point_to_pointmsg(point):
    ros_point = Point()
    ros_point.x = point[0]
    ros_point.y = point[1]
    ros_point.z = point[2]
    return ros_point 


class PointcloudProcess:
    """
    Wraps the processing of a pointcloud from an input ros topic and publishing
    to another PointCloud2 topic.

    """
    def __init__(self, points_sub_topic, 
                       image_sub_topic,
                       depth_sub_topic,
                       cam_info_topic,
                       points_pub_topic):

        self.num_steps = 0

        self.messages = deque([], 5)
        self.pointcloud_frame = None
        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        depth_sub = message_filters.Subscriber(depth_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        #A1 Soccer
        self.cv_bridge = CvBridge()
        self.latest_depth_im = None
        self.listener = tf.TransformListener()
        self.max_center_point = None
        self.max_contour_area = None
        
        self.points_pub = rospy.Publisher(points_pub_topic, Point, queue_size=10)
        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, depth_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)

        ts.registerCallback(self.callback)

    def isolate_object_of_interest(self, points, image, camera_info, trans, rot):

        depth = self.latest_depth_im.copy()

        show_image(image)

        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space
        greenLower = (10, 105, 123)
        greenUpper = (122, 249, 255)
        # greenLower = (29, 86, 6)
        # greenUpper = (64, 255, 255)

        # # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(image, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        #show_image(mask, title="orig")
        mask = cv2.erode(mask, None, iterations=2)
        #show_image(mask, title="erode")
        mask = cv2.dilate(mask, None, iterations=2)
        #show_image(mask, title="dilate")

        #test_thresh_naive2(image, 0,130, 70,220, 100,220)

        #mask = segment_image(image)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        print("length of cnts:", len(cnts))
        #for c in cnts:
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            contour = max(cnts, key=cv2.contourArea)

            #filter contours that have area > 0.6*Area of max contour
            #filt_cnts = [cnt for cnt in contours if cv.contourArea(cnt)>0.6*cv.contourArea(outer_cnt)]

            # if not min_radius_circle < radius < max_radius_circle:
            #   continue

            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > 10:
                M = cv2.moments(contour)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                print("Center:", center)
                x, y = center

                show_image_with_point(image, center)

                depth_val = None
                try:
                    depth_val = depth[x,y]
                    center_soccer_point_3d = convert_depth_to_phys_coord_using_realsense(x, y, depth_val, camera_info)
                    print(center_soccer_point_3d)
                    return center_soccer_point_3d, contour
                except:
                    print("Center of ball not in frame. Skipping")
            else:
                print("Contour found is too small. Skipping")
        else:
            print("No contours found. Skipping")

        return None

        ### Old code
            # points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
            # return points
    
    def callback(self, points_msg, image, depth, info): 
        try:
            intrinsic_matrix = get_camera_matrix(info)
            intrinsic_matrix = info
            rgb_image = ros_numpy.numpify(image)
            points = ros_numpy.numpify(points_msg)

            ### A1 SOCCER ###
            depth_image = self.cv_bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            depth_array = np.array(depth_image, dtype=np.float32)
            self.latest_depth_im = depth_array

        except Exception as e:
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft((points, rgb_image, intrinsic_matrix))

    def publish_once_from_queue(self):
        if self.messages:
            points, image, info = self.messages.pop()
            try:
                trans, rot = self.listener.lookupTransform(
                                                       '/camera_color_optical_frame',
                                                       '/camera_depth_optical_frame',
                                                       rospy.Time(0))
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException):
                return

            ### A1 SOCCER ###
            center_soccer_point, contour = self.isolate_object_of_interest(points, image, info, np.array(trans), np.array(rot))
            self.max_center_point = center_soccer_point
            
            ### TODO: save the largest contour and associated 3d point. Only update when a larger one has been found! Continuously publish this point###

            #self.max_center_point = center_soccer_point if center_soccer_point > self.max_center_point else self.max_center_point

            # TODO: Modify center point given ARTag x location

            # pointmsg = point_to_pointmsg(self.max_center_point)
            # self.points_pub.publish(pointmsg)
            print("Publishing soccer point:", self.max_center_point, contour)

def main():
    CAM_INFO_TOPIC = '/camera/aligned_depth_to_color/camera_info' #'/camera/color/camera_info'
    #CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    DEPTH_IMAGE_TOPIC = '/camera/aligned_depth_to_color/image_raw' #'/camera/depth/image_rect_raw'
    #DEPTH_IMAGE_TOPIC = '/camera/depth/image_rect_raw'
    POINTS_TOPIC = '/camera/depth/color/points'
    POINTS_PUB_TOPIC = 'segmented_points'

    rospy.init_node('realsense_listener')
    process = PointcloudProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC, DEPTH_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    r = rospy.Rate(1000)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':
    main()
