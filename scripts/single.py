#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue May 19 00:28:30 2020

@author: mason
"""

''' import libraries '''
import time
import numpy as np
import utm

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def tic():
    return time.time()
def toc(t): 
    return float(time.time() - t)

''' class '''

class path_pub():
    def __init__(self):
        rospy.init_node('path_pubb', anonymous=True)
        self.parent_frame_id = rospy.get_param("~parent_frame_id", 'map')
        self.enable_altitude = rospy.get_param("/enable_altitude", False)
        
        self.input_topic_name = rospy.get_param("~input_topic_name", '/fix')
        self.out_pose_topic_name = rospy.get_param("~out_pose_topic_name", '/pose')
        self.out_path_topic_name = rospy.get_param("~out_path_topic_name", '/path')

        rospy.Subscriber(self.input_topic_name, NavSatFix, self.navsatfix_cb)
        self.pose_pub = rospy.Publisher(self.out_pose_topic_name, PoseStamped, queue_size=3)
        self.path_pub = rospy.Publisher(self.out_path_topic_name, Path, queue_size=3)
        self.path = Path()
        self.offset=[]
        self.append_counter = 0

        self.rate = rospy.Rate(5)

    def navsatfix_cb(self, msg):
        time_start = tic()
        if len(self.offset)==0:
            temp_tuple=utm.from_latlon(msg.latitude, msg.longitude)
            self.offset=[temp_tuple[0], temp_tuple[1], msg.altitude]
            return
        else:
            self.append_counter += 1
            if self.append_counter % 10 == 0:
                temp_tuple=utm.from_latlon(msg.latitude, msg.longitude)
                pose = PoseStamped()
                pose.header.stamp = msg.header.stamp
                pose.header.frame_id = self.parent_frame_id
                pose.pose.position.x = temp_tuple[0]  - self.offset[0]
                pose.pose.position.y = temp_tuple[1]  - self.offset[1]
                z_value = msg.altitude - self.offset[2]
                if self.enable_altitude:
                    pose.pose.position.z = z_value
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)

                if self.append_counter % 50 == 0:
                    self.path.poses.append(pose)
                    self.path.header.stamp = rospy.Time.now()
                    self.path.header.frame_id = self.parent_frame_id
                    self.path_pub.publish(self.path)
        time_duration = toc(time_start)
        # print("time: ", time_duration)

''' main '''
path_pub_class = path_pub()

if __name__ == '__main__':
    while 1:
        try:
            path_pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
