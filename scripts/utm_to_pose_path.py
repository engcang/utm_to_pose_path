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


''' class '''

class path_pub():
    def __init__(self):
        rospy.init_node('path_pubb', anonymous=True)
        self.parent_frame_id = rospy.get_param("~parent_frame_id", 'map')
        self.enable_altitude = rospy.get_param("/enable_altitude", False)
        self.enable_file_write = rospy.get_param("/enable_file_write", False)
        self.file_folder = rospy.get_param("/file_folder", "")
        
        self.input_topic_name1 = rospy.get_param("~input_topic_name1", '/fix')
        self.out_pose_topic_name1 = rospy.get_param("~out_pose_topic_name1", '/pose1')
        self.out_path_topic_name1 = rospy.get_param("~out_path_topic_name1", '/path1')

        self.input_topic_name2 = rospy.get_param("~input_topic_name2", '/one/mavros/global_position/raw/fix')
        self.out_pose_topic_name2 = rospy.get_param("~out_pose_topic_name2", '/pose2')
        self.out_path_topic_name2 = rospy.get_param("~out_path_topic_name2", '/path2')

        self.input_topic_name3 = rospy.get_param("~input_topic_name3", '/two/mavros/global_position/raw/fix')
        self.out_pose_topic_name3 = rospy.get_param("~out_pose_topic_name3", '/pose3')
        self.out_path_topic_name3 = rospy.get_param("~out_path_topic_name3", '/path3')

        rospy.Subscriber(self.input_topic_name1, NavSatFix, self.navsatfix_cb1)
        self.pose_pub1 = rospy.Publisher(self.out_pose_topic_name1, PoseStamped, queue_size=3)
        self.path_pub1 = rospy.Publisher(self.out_path_topic_name1, Path, queue_size=3)
        self.path1 = Path()
        self.offset1=[]

        rospy.Subscriber(self.input_topic_name2, NavSatFix, self.navsatfix_cb2)
        self.pose_pub2 = rospy.Publisher(self.out_pose_topic_name2, PoseStamped, queue_size=3)
        self.path_pub2 = rospy.Publisher(self.out_path_topic_name2, Path, queue_size=3)
        self.path2 = Path()
        self.offset2=[]

        rospy.Subscriber(self.input_topic_name3, NavSatFix, self.navsatfix_cb3)
        self.pose_pub3 = rospy.Publisher(self.out_pose_topic_name3, PoseStamped, queue_size=3)
        self.path_pub3 = rospy.Publisher(self.out_path_topic_name3, Path, queue_size=3)
        self.path3 = Path()
        self.offset3=[]

        self.rate = rospy.Rate(5)


    def navsatfix_cb1(self, msg):
        if len(self.offset1)==0:
            temp_tuple=utm.from_latlon(msg.latitude, msg.longitude)
            self.offset1=[temp_tuple[0], temp_tuple[1], msg.altitude]
            return
        else:
            temp_tuple=utm.from_latlon(msg.latitude, msg.longitude)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.parent_frame_id
            pose.pose.position.x = temp_tuple[0]  - self.offset1[0]
            pose.pose.position.y = temp_tuple[1]  - self.offset1[1]
            z_value=msg.altitude - self.offset1[2]
            if self.enable_altitude:
                pose.pose.position.z = z_value
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            if self.enable_file_write:
                self.f1 = open(self.file_folder+self.out_path_topic_name1+'.csv', "a")
                self.f1.write("%.6f %.6f %.6f %.6f\n"%( (msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000.0) , pose.pose.position.x, pose.pose.position.y, z_value))
                self.f1.close()

            self.pose_pub1.publish(pose)
            self.path1.poses.append(pose)
            self.path1.header.stamp = rospy.Time.now()
            self.path1.header.frame_id = self.parent_frame_id
            self.path_pub1.publish(self.path1)
        
    def navsatfix_cb2(self, msg):
        if len(self.offset2)==0:
            temp_tuple=utm.from_latlon(msg.latitude, msg.longitude)
            self.offset2=[temp_tuple[0], temp_tuple[1], msg.altitude]
            return
        else:
            temp_tuple=utm.from_latlon(msg.latitude, msg.longitude)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.parent_frame_id
            pose.pose.position.x = temp_tuple[0]  - self.offset2[0]
            pose.pose.position.y = temp_tuple[1]  - self.offset2[1]
            z_value=msg.altitude - self.offset2[2]
            if self.enable_altitude:
                pose.pose.position.z = z_value
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            if self.enable_file_write:
                self.f2 = open(self.file_folder+self.out_path_topic_name2+'.csv', "a")
                self.f2.write("%.6f %.6f %.6f %.6f\n"%( (msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000.0) , pose.pose.position.x, pose.pose.position.y, z_value))
                self.f2.close()

            self.pose_pub2.publish(pose)
            self.path2.poses.append(pose)
            self.path2.header.stamp = rospy.Time.now()
            self.path2.header.frame_id = self.parent_frame_id
            self.path_pub2.publish(self.path2)

    def navsatfix_cb3(self, msg):
        if len(self.offset3)==0:
            temp_tuple=utm.from_latlon(msg.latitude, msg.longitude)
            self.offset3=[temp_tuple[0], temp_tuple[1], msg.altitude]
            return
        else:
            temp_tuple=utm.from_latlon(msg.latitude, msg.longitude)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.parent_frame_id
            pose.pose.position.x = temp_tuple[0]  - self.offset3[0]
            pose.pose.position.y = temp_tuple[1]  - self.offset3[1]
            z_value=msg.altitude - self.offset3[2]
            if self.enable_altitude:
                pose.pose.position.z = z_value            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            if self.enable_file_write:
                self.f3 = open(self.file_folder+self.out_path_topic_name3+'.csv', "a")
                self.f3.write("%.6f %.6f %.6f %.6f\n"%( (msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000.0) , pose.pose.position.x, pose.pose.position.y, z_value))
                self.f3.close()

            self.pose_pub3.publish(pose)
            self.path3.poses.append(pose)
            self.path3.header.stamp = rospy.Time.now()
            self.path3.header.frame_id = self.parent_frame_id
            self.path_pub3.publish(self.path3)

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
