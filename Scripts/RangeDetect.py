#! /usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math as m
import numpy as np
from PreDefine import *

class RangeDetect:
    def __init__(self):
        self.scan_range_ = 0
        self.scan_ang_ = 0
        self.ang_z_ = 0
        self.pos_x_ = 0
        self.pos_y_ = 0
        self.x_wall = np.zeros([GRID_SIZE+1, GRID_SIZE])
        self.y_wall = np.zeros([GRID_SIZE, GRID_SIZE+1])
        self.max_dist_ = 3.5

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scanCallback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size=1)
        self.range_pub = rospy.Publisher('/range_pub', Float32MultiArray, queue_size=1)
        self.x_wall_pub = rospy.Publisher('/x_wall_pub', Int16MultiArray, queue_size=1)
        self.y_wall_pub = rospy.Publisher('/y_wall_pub', Int16MultiArray, queue_size=1)

        rospy.loginfo('RangeDetect Node Initialized')    

    def scanCallback(self,msg):
        scan_data = np.asarray(msg.ranges)
        scan_increment = msg.angle_increment
        scan_size = scan_data.size

        for i in range (0,scan_size):
            if m.isinf(scan_data[i]):
                continue
            angle = scan_increment*i

            if angle > PI:
                angle -= 2*PI

            x = scan_data[i]*m.cos(angle)
            y = scan_data[i]*m.sin(angle)
            x_inertia = x*m.cos(self.ang_z_) - y*m.sin(self.ang_z_)
            y_inertia = x*m.sin(self.ang_z_) - y*m.sin(self.ang_z_)

            x_inertia += self.pos_x_
            y_inertia += self.pos_y_

            if x_inertia < 0 or y_inertia < 0:
                continue

            if m.fmod(x_inertia, 0.5) > 0.35 or m.fmod(x_inertia, 0.5) < 0.15:
                x_idx = int((round(x_inertia/0.5)*5)/5)
                y_idx = int((round(y_inertia/0.5)*5)/5)
                if x_idx%2 == 0 and y_idx%2:
                    self.x_wall[int(x_idx/2)][int((y_idx-1)/2)] = 1
                if x_idx%2 and y_idx%2 == 0:
                    self.y_wall[int((x_idx-1)/2)][int(y_idx/2)] = 1


        smallest_dist = 100
        for i in range(0, scan_size):
                if scan_data[i] < smallest_dist:
                    smallest_dist = scan_data[i]
                    self.scan_ang_ = msg.angle_min + msg.angle_increment*i
                    if self.scan_ang_ > PI:
                        self.scan_ang_ -= 2*PI
        self.scan_range = smallest_dist

        self.pub()
        
        rospy.loginfo("---------------------------------")
        rospy.loginfo("Nearest obstace distance: %.2f", self.scan_range_)
        rospy.loginfo("Occupancy Grid")
        for i in range(9,0,-1):
            if i != 9:
                rospy.loginfo("%s %s %s %s %s %s %s %s %s %s",
                str(self.drawwall(self.y_wall[i,9],1)),
                str(self.drawwall(self.y_wall[i,8],1)),
                str(self.drawwall(self.y_wall[i,7],1)),
                str(self.drawwall(self.y_wall[i,6],1)),
                str(self.drawwall(self.y_wall[i,5],1)),
                str(self.drawwall(self.y_wall[i,4],1)),
                str(self.drawwall(self.y_wall[i,3],1)),
                str(self.drawwall(self.y_wall[i,2],1)),
                str(self.drawwall(self.y_wall[i,1],1)),
                str(self.drawwall(self.y_wall[i,0],1)),
                )
            rospy.loginfo("%s %s %s %s %s %s %s %s %s",
                str(self.drawwall(self.x_wall[i,8],0)),
                str(self.drawwall(self.x_wall[i,7],0)),
                str(self.drawwall(self.x_wall[i,6],0)),
                str(self.drawwall(self.x_wall[i,5],0)),
                str(self.drawwall(self.x_wall[i,4],0)),
                str(self.drawwall(self.x_wall[i,3],0)),
                str(self.drawwall(self.x_wall[i,2],0)),
                str(self.drawwall(self.x_wall[i,1],0)),
                str(self.drawwall(self.x_wall[i,0],0)),
            )

    
    def odomCallback(self,msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.ang_z_ = m.atan2(2*(qw*qz + qx*qy), 1-2*(qz*qz+qy*qy))

        self.pos_x_ = msg.pose.pose.position.x
        self.pos_y_ = msg.pose.pose.position.y
    
    def pub(self):
        x_pub = Int16MultiArray()
        y_pub = Int16MultiArray()

        x_pub.data.clear()
        y_pub.data.clear()

        for i in range(0,GRID_SIZE):
            for j in range(0, GRID_SIZE+1):
                x_pub.data.append(self.x_wall[j][i])
        for i in range(0, GRID_SIZE+1):
            for j in range(0, GRID_SIZE):
                y_pub.data.append(self.y_wall[j][i])
        
        self.x_wall_pub.publish(x_pub)
        self.y_wall_pub.publish(y_pub)

        msg_pub = Float32MultiArray()
        msg_pub.data.append(self.scan_range_)
        msg_pub.data.append(self.scan_ang_)
        self.range_pub.publish(msg_pub)

    def drawwall(self,draw, xy):
        if draw and not xy:
            return "-"
        elif draw and xy:
            return "|"
        else:
            return " "



if __name__=='__main__':
    rospy.init_node('range_detect_node')
    detector = RangeDetect()
    rospy.spin()
