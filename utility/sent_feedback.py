#!/usr/bin/env python

import socket
import rospy
from geometry_msgs.msg import PoseStamped
import json
from threading import Thread
import time
import math
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
import sys
import numpy as np
import time

class KalmanFilter(object):

    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate

process_variance = eval(sys.argv[3]) or 1e-3 #Q
estimated_measurement_variance =  eval(sys.argv[2]) or 0.5 ** 2# 0.05 ** 2 #R
kalman_filter_x = KalmanFilter(process_variance, estimated_measurement_variance)
kalman_filter_y = KalmanFilter(process_variance, estimated_measurement_variance)
kalman_filter_z = KalmanFilter(process_variance, estimated_measurement_variance)

print "----Kalman filtering----"
print "Details:"
print "R =", estimated_measurement_variance
print "Q =", process_variance
print "------------------------"

UDP_IP = sys.argv[1] or "192.168.0.2"
UDP_PORT = 8080
msg = PoseStamped()
MESSAGE = {'header': {'seq': msg.header.seq, 'stamp': msg.header.stamp.nsecs, 'frame_id': msg.header.frame_id}, 
           'pose': {'position': {'x': 0 , 'y': 0, 'z': 0}, 
           'orientation': {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y, 'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}}}
IN_MESSAGE = [MESSAGE, MESSAGE]
flag = False
print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

rospy.init_node('image_converter', anonymous=True)
socket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
socket.bind(("0.0.0.0", 8080))
def feedback(msg):
    data = {'header': {'seq': msg.header.seq, 'stamp': msg.header.stamp.nsecs, 'frame_id': msg.header.frame_id}, 
           'pose': {'position': {'x': msg.pose.position.y, 'y': msg.pose.position.z, 'z': msg.pose.position.x}, 
           'orientation': {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y, 'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}}}
    global IN_MESSAGE, flag
    kalman_filter_x.input_latest_noisy_measurement( data['pose']['position']['x'] )
    kalman_filter_y.input_latest_noisy_measurement( data['pose']['position']['y'] )
    kalman_filter_z.input_latest_noisy_measurement( data['pose']['position']['z'] )

    data['pose']['position']['x'] = kalman_filter_x.get_latest_estimated_measurement()
    data['pose']['position']['y'] = kalman_filter_y.get_latest_estimated_measurement()
    data['pose']['position']['z'] = kalman_filter_z.get_latest_estimated_measurement()
    IN_MESSAGE[1] = data

#http://paulbourke.net/miscellaneous/interpolation/
def cosine_interpolate(y1, y2, mu):
   mu2 = (1-math.cos(mu*math.pi))/2
   return (y1*(1-mu2)+y2*mu2)

def linear_interpolate(y1, y2, mu):
   return (y1*(1-mu)+y2*mu)

def send_data():
  seq = 0
  global count, flag
  pubX = rospy.Publisher('debug/x', Float64, queue_size=10)
  pubY = rospy.Publisher('debug/y', Float64, queue_size=10)
  pubZ = rospy.Publisher('debug/z', Float64, queue_size=10)
  start_time = time.time()
  while True:
    _time = rospy.Time.now()
    data = IN_MESSAGE[1]
    data['header']['seq'] = seq
    data['pose']['position']['x'] = data['pose']['position']['x']
    data['pose']['position']['y'] = data['pose']['position']['y']
    data['pose']['position']['z'] = data['pose']['position']['z']

    json_data = json.dumps(data)
    socket.sendto(json_data, (UDP_IP, UDP_PORT))

    elapsed_time = time.time() - start_time
    sys.stdout.write("\r" + "{0:>75} Hz".format(str(int(seq/elapsed_time*1000)/1000.0)))
    sys.stdout.write("\r" + "{0:>65}".format(str(seq)))
    sys.stdout.write("\r" + "{0:>55}".format("SEQ: "))
    sys.stdout.write("\r" + "#"*(seq%60))
    sys.stdout.flush()

    pubX.publish(data=(data['pose']['position']['x']))
    pubY.publish(data=(data['pose']['position']['y']))
    pubZ.publish(data=(data['pose']['position']['z']))
    
    seq += 1
    time.sleep(0.067)

t1 = Thread(target = send_data)
t1.setDaemon(True)
t1.start()
data_sub = rospy.Subscriber("/ORB_SLAM/pose", PoseStamped, feedback)
rospy.spin()
