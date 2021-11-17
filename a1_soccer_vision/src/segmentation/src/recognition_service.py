#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import PIL.Image as Image
import PIL.ImageOps as ImageOps
from button_recognition.msg import recognition
from button_recognition.msg import recog_result
from button_recognition.srv import *
from ocr_rcnn_lib.button_recognition import ButtonRecognizer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Image_msg
from sensor_msgs.msg import CameraInfo
import pyrealsense2 as rs2
from visualization_msgs.msg import Marker




def resize_to_480x680(img):
    print("shape of panel image input: {}".format(img.shape))
    if img.shape != (480, 640):
        img_pil = Image.fromarray(img)
        img_thumbnail = img_pil.thumbnail((640, 480), Image.ANTIALIAS)
        delta_w, delta_h = 640 - img_pil.size[0], 480 - img_pil.size[1]
        padding = (delta_w // 2, delta_h // 2, delta_w - (delta_w // 2), delta_h - (delta_h // 2))
        new_im = ImageOps.expand(img_pil, padding)
        img = np.copy(np.asarray(new_im))

    print('result shape after resize', img.shape)
    return img

class RecognitionService:
  def __init__(self, model):
    self.model = model
    assert isinstance(self.model, ButtonRecognizer)
    self.cv_bridge = CvBridge()
    self.latest_im = None
    rospy.Subscriber("/camera/color/image_raw", Image_msg, self.im_callback)

    self.latest_depth_im = None
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image_msg, self.depth_callback)

    self.intrinsics = None
    rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.imageDepthInfoCallback)

    self.marker_pub = rospy.Publisher('/button', Marker, queue_size=1, latch=True)



  def depth_callback(self, data):
    depth_image = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    depth_array = np.array(depth_image, dtype=np.float32)
    self.latest_depth_im = depth_array

  def im_callback(self, data):
    cv_image = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    self.latest_im = cv_image

  def imageDepthInfoCallback(self, cameraInfo):
    try:
      if self.intrinsics:
        return
      self.intrinsics = rs2.intrinsics()
      self.intrinsics.width = cameraInfo.width
      self.intrinsics.height = cameraInfo.height
      self.intrinsics.ppx = cameraInfo.K[2]
      self.intrinsics.ppy = cameraInfo.K[5]
      self.intrinsics.fx = cameraInfo.K[0]
      self.intrinsics.fy = cameraInfo.K[4]
      if cameraInfo.distortion_model == 'plumb_bob':
        self.intrinsics.model = rs2.distortion.brown_conrady
      elif cameraInfo.distortion_model == 'equidistant':
        self.intrinsics.model = rs2.distortion.kannala_brandt4
      self.intrinsics.coeffs = [i for i in cameraInfo.D]
    except CvBridgeError as e:
      print(e)
      return

  def perform_recognition(self, request):
    button_text = request.buttonText
    start = rospy.get_time()

    video = False
    ##should change to subscriber
    # image_name = "/home/zixianzang/Desktop/bag_image/170.jpg"
    # im = cv2.imread(image_name)
    #
    # depth_name = "/home/zixianzang/Desktop/bag_depth_images/170.npy"
    # depth = np.load(depth_name)







    im = self.latest_im.copy()
    depth = self.latest_depth_im.copy()
    image_np = resize_to_480x680(im.copy())

    recognitions = self.model.predict(image_np)

    _ = self.model.predict(image_np, draw=True)
    cv2.imwrite("/home/zixianzang/Desktop/result.jpg", image_np)

    if video:
      for i in range(600, 1100):
        print(i)
        im_dir = "/home/zixianzang/Desktop/bag_image/{}.jpg".format(str(i))
        im_ = cv2.imread(im_dir, cv2.IMREAD_COLOR)
        image_np_ = resize_to_480x680(im_.copy())
        _ = self.model.predict(image_np_, draw=True)
        cv2.imwrite("/home/zixianzang/Desktop/detected_image/{}.jpg".format(str(i)), image_np_)

    recog_resp = recog_result()
    exist = False

    #hack implementation for up and down
    if len(recognitions) == 2:
      y_min_1 = recognitions[0][0][0]
      y_min_2 = recognitions[1][0][0]
      if y_min_2 > y_min_1:
        recognitions[1][2] = ')'
        recognitions[0][2] = '('
      elif y_min_2 < y_min_1:
        recognitions[1][2] = '('
        recognitions[0][2] = ')'


    for item in recognitions:
      if item[2] == button_text:
        sample = recognition()
        ymin = int(item[0][0] * image_np.shape[0])
        xmin = int(item[0][1] * image_np.shape[1])
        ymax = int(item[0][2] * image_np.shape[0])
        xmax = int(item[0][3] * image_np.shape[1])
        print(ymin, ymax)
        exist = True

        originHeight, originWidth = float(im.shape[0]), float(im.shape[1])
        ymin, ymax = float(ymin), float(ymax)
        xmin, xmax = float(xmin), float(xmax)

        if originHeight/480 > originWidth/640:
          print('tall')
          shrink_ratio = originHeight/480
          origin_ymin, origin_ymax = int(ymin*shrink_ratio), int(ymax*shrink_ratio)
          xpad = (640 - originWidth / shrink_ratio)/2
          origin_xmin, origin_xmax = int((xmin-xpad)*shrink_ratio), int((xmax-xpad)*shrink_ratio)
        elif originHeight/480 < originWidth/640:
          print('wide')
          shrink_ratio = originWidth / 640
          origin_xmin, origin_xmax = int(xmin * shrink_ratio), int(xmax * shrink_ratio)
          ypad = (480 - originHeight / shrink_ratio) / 2
          origin_ymin, origin_ymax = int((ymin - ypad) * shrink_ratio), int((ymax - ypad) * shrink_ratio)

        else:
          print('same ratio')
          shrink_ratio = originWidth / 640
          origin_xmin, origin_xmax = int(xmin * shrink_ratio), int(xmax * shrink_ratio)
          origin_ymin, origin_ymax = int(ymin * shrink_ratio), int(ymax * shrink_ratio)

        sample.score = item[1]  # float
        sample.text = item[2]
        sample.belief = item[3]
        sample.x_min = origin_xmin
        sample.y_min = origin_ymin
        sample.x_max = origin_xmax
        sample.y_max = origin_ymax

        x_mid = int((origin_xmin+origin_xmax) / 2)
        y_mid = int((origin_ymin + origin_ymax) / 2)

        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [x_mid, y_mid], depth[y_mid, x_mid])
        print(result)
        x = result[0]
        y = result[1]
        z = result[2]
        # print(x, y, z)
        sample.x_3d = x
        sample.y_3d = y
        sample.z_3d = z
        recog_resp.data.append(sample)

        show_marker = True
        if show_marker:
          for iter in range(50):
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = rospy.get_rostime()
            marker.id = 0
            marker.type = 2  # sphere
            marker.action = 0
            marker.pose.position.x = result[2]/1000
            marker.pose.position.y = -result[0]/1000
            marker.pose.position.z = -result[1]/1000
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime = rospy.Duration(0)
            self.marker_pub.publish(marker)
            rospy.sleep(0.1)

    ###visualize
    # for b in recog_resp.data:
    #   copy = im.copy()
    #   im[b.y_min:b.y_max, b.x_min:b.x_max, 0] = 0
    #   im[b.y_min:b.y_max, b.x_min:b.x_max, 1] = 255
    #   im[b.y_min:b.y_max, b.x_min:b.x_max, 2] = 0
    #   linewidth = 5
    #   im[b.y_min+linewidth:b.y_max-linewidth] = copy[b.y_min+linewidth:b.y_max-linewidth]
    #   cv2.imshow("detection", im)
    #   cv2.waitKey(0)

    # ###debug
    # if exist:
    #   for b in recog_resp.data:
    #
    #     originHeight, originWidth = float(im.shape[0]), float(im.shape[1])
    #     print(originHeight, originWidth)
    #
    #     ymin, ymax = float(b.y_min), float(b.y_max)
    #     xmin, xmax = float(b.x_min), float(b.x_max)
    #     #taller
    #     if originHeight/480 > originWidth/640:
    #       print('tall')
    #       shrink_ratio = originHeight/480
    #       origin_ymin, origin_ymax = int(ymin*shrink_ratio), int(ymax*shrink_ratio)
    #       xpad = (640-originWidth/shrink_ratio)/2
    #       origin_xmin, origin_xmax = int((xmin-xpad)*shrink_ratio), int((xmax-xpad)*shrink_ratio)
    #
    #     # im[origin_ymin:origin_ymax, origin_xmin:origin_xmax] = 0
    #     # print('origin_cor', origin_ymin, origin_ymax, origin_xmin, origin_xmax)
    #     # im[0:50, 50:100] = 0
    #     # cv2.imwrite("/home/zixianzang/Desktop/pos_origin.jpg", im)

    # ###debug
    # if exist:
    #   for b in recog_resp.data:
    #     image_np[b.y_min:b.y_max, b.x_min:b.x_max] = 0
    #     print(b.y_min, b.y_max, b.x_min, b.x_max)
    #     cv2.imwrite("/home/zixianzang/Desktop/pos.jpg", image_np)


    if not exist:
      rospy.loginfo('no button with text {} found'.format(button_text))
      print('found', [i[2] for i in recognitions])
    end = rospy.get_time()
    rospy.loginfo('Recognition finished: {} buttons are detected using {} seconds!'.format(
      len(recognitions), end-start))
    return recog_serverResponse(recog_resp)


def button_recognition_server():
  rospy.init_node('button_recognition_server', anonymous=True)
  model = ButtonRecognizer(use_optimized=True)
  recognizer = RecognitionService(model)
  service = rospy.Service('recognition_service',
                          recog_server,
                          recognizer.perform_recognition)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    recognizer.model.clear_session()
    rospy.logdebug('Shutting down ROS button recognition module!')
  cv2.destroyAllWindows()


if __name__ == '__main__':
  button_recognition_server()
