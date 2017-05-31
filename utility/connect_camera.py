#!/usr/bin/env python

import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import time
import sys
import urllib2

rospy.init_node('image_converter', anonymous=True)
ip = sys.argv[1] or "192.168.0.2"
ip += ":8080"
print "Connect to " + ip
cap = cv2.VideoCapture('http://'+ ip +'/video')
image_pub = rospy.Publisher("/camera/image_raw",Image, queue_size = 0)
bridge = CvBridge()
print "Connected:", cap.isOpened()
start_time = time.time()
seq = 0
start_time = time.time()
while(cap.isOpened()):
    seq += 1
    flag, frame = cap.read()
    frame = cv2.GaussianBlur(frame, (5,5), 0.6)
    image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

    elapsed_time = time.time() - start_time
    sys.stdout.write("\r" + "{0:>75} Hz".format(str(int(seq/elapsed_time*1000)/1000.0)))
    sys.stdout.write("\r" + "{0:>65}".format(str(seq)))
    sys.stdout.write("\r" + "{0:>55}".format("SEQ: "))
    sys.stdout.write("\r" + "#"*(seq%60))
    sys.stdout.flush()


cap.release()
cv2.destroyAllWindows()

#
# stream=urllib2.urlopen('http://'+ ip +'/video')
#
# bytes=''
# start_time = time.time()
# seq = 0
# while True:
#     bytes+=stream.read(1024)
#     a = bytes.find('\xff\xd8')
#     b = bytes.find('\xff\xd9')
#     if a!=-1 and b!=-1:
#         jpg = bytes[a:b+2]
#         bytes= bytes[b+2:]
#         i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
#         image_pub.publish(bridge.cv2_to_imgmsg(i, "bgr8"))
#         seq += 1
#         elapsed_time = time.time() - start_time
#         sys.stdout.write("\r" + "{0:>75} Hz".format(str(int(seq/elapsed_time*1000)/1000.0)))
#         sys.stdout.write("\r" + "{0:>65}".format(str(seq)))
#         sys.stdout.write("\r" + "{0:>55}".format("SEQ: "))
#         sys.stdout.write("\r" + "#"*(seq%60))
#         sys.stdout.flush()
