#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import argparse

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Char

pub = rospy.Publisher('cyan3/vision_ball_sim', Char, queue_size=10)

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("cyan3/omni_cam/omni_image_raw", Image, self.callbackData)

  def callbackData(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    
    blank = np.zeros((720,720,3), dtype='uint8')
    image = cv_image

    resized_image = cv2.resize(image, (720, 720)) 
    image = resized_image
    frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # thresh = cv2.inRange(frame_to_thresh, (0, 58, 0), (22, 255, 255))
    thresh = cv2.inRange(frame_to_thresh, (0, 58, 0), (15, 255, 255))
    _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    jumlah = 0

    for i in range(len(contours)):
      cnt = contours[i]
      # print(len(contours))
      (x,y),radius = cv2.minEnclosingCircle(cnt)
      center = (int(x),int(y))
        #Jika radius(kontur ukuran > 1 ) diaanggap kontur jika kurang bukan kontur
      if(int(radius) > 1) :
          contours.append(cnt)
#           #Gambar lingkaran
          cv2.circle(blank,center,int(radius),(0,0,255),1)
          cv2.circle(blank,center,1,(0,255,0),1)

          jumlah = 1

    pesan = Char()
    pesan.data = jumlah
    pub.publish(pesan)
    # cv2.imshow("Thresh", thresh)
    # cv2.imshow("Normal", resized_image)
    # cv2.imshow("bola", blank)
    cv2.waitKey(3)
  
def callback(value):
  pass

def main():
	camera_1()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()