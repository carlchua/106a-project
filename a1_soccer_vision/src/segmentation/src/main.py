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

from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import numpy as np
import cv2

from cv_bridge import CvBridge

from image_segmentation import segment_image
from pointcloud_segmentation import segment_pointcloud


def get_camera_matrix(camera_info_msg):
    # TODO: Return the camera intrinsic matrix as a 3x3 numpy array
    # by retreiving information from the CameraInfo ROS message.
    # Hint: numpy.reshape may be useful here.

    print(np.reshape(camera_info_msg.K, (3,3)))

    return np.reshape(camera_info_msg.K, (3,3))

def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):
    _intrinsics = pyrealsense2.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = pyrealsense2.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]
    result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
    #result[0]: right, result[1]: down, result[2]: forward
    return result[2], -result[0], -result[1]

def isolate_object_of_interest(points, image, camera_info, trans, rot):
    print("test3")
    depth = self.latest_depth_im.copy()
    mask = segment_image(image)

    # find contours in the mask and initialize the current
	# (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        print("Center:", center)
		# only proceed if the radius meets a minimum size
        ### TODO: Double check that this makes sense for us
		#if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			# cv2.circle(frame, (int(x), int(y)), int(radius),
			# 	(0, 255, 255), 2)
			# cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # x, y = center
        # convert_depth_to_phys_coord_using_realsense(x, y, depth[x,y], camera_info)

    # points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
    # return points

### A1 SOCCER ###

# def isolate_soccer_ball(points, image, cam_matrix, trans, rot):
#     segmented_image = segment_image(image)
#     #points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
#     #return points
#     # For each point x,y and depth d
#     for x,y in segmented_image:
#         convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo)

#         xyz = np.vstack((points['x'], points['y'], points['z']))
#     pixel_coords = project_points(xyz, cam_matrix, trans, rot)

# def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):
#     _intrinsics = pyrealsense2.intrinsics()
#     _intrinsics.width = cameraInfo.width
#     _intrinsics.height = cameraInfo.height
#     _intrinsics.ppx = cameraInfo.K[2]
#     _intrinsics.ppy = cameraInfo.K[5]
#     _intrinsics.fx = cameraInfo.K[0]
#     _intrinsics.fy = cameraInfo.K[4]
#     #_intrinsics.model = cameraInfo.distortion_model
#     _intrinsics.model  = pyrealsense2.distortion.none
#     _intrinsics.coeffs = [i for i in cameraInfo.D]
#     result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
#     #result[0]: right, result[1]: down, result[2]: forward
#     return result[2], -result[0], -result[1]

def numpy_to_pc2_msg(points):
    return ros_numpy.msgify(PointCloud2, points, stamp=rospy.Time.now(),
        frame_id='camera_depth_optical_frame')

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
        self.latest_depth_im = None
        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        depth_sub = message_filters.Subscriber(depth_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        self.points_pub = rospy.Publisher(points_pub_topic, PointCloud2, queue_size=10)
        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, depth_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, points_msg, image, depth, info):
        print("test2")
        try:
            intrinsic_matrix = get_camera_matrix(info)
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
            points, image, camera_info = self.messages.pop()
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
            # points = isolate_object_of_interest(points, image, info, 
            #     np.array(trans), np.array(rot))

            ### A1 SOCCER ###
            points = isolate_object_of_interest(points, image, camera_info, np.array(trans), np.array(rot))

            #points_msg = numpy_to_pc2_msg(points)
            #self.points_pub.publish(points_msg)
            #print("Published segmented pointcloud at timestamp:",
                   #points_msg.header.stamp.secs)

def main():
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    DEPTH_IMAGE_TOPIC = '/camera/aligned_depth_to_color/image_raw'
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
