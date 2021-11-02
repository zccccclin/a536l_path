#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math as m
import numpy as np
import SpiralPath
from PreDefine import *

class PathPlan:
    def __init__(self):
        self.pos_x_ = 0
        self.pos_y_ = 0
        self.target_x = 0
        self.target_y = 0
        self.target_heading = 0
        self.goal_reached = GOAL_NOT_REACH
        self.x_wall = np.zeros([GRID_SIZE+1, GRID_SIZE],dtype=int)
        self.y_wall = np.zeros([GRID_SIZE, GRID_SIZE+1],dtype=int)
        self.dist_to_center = 0.5
        self.tgt_idx = -1
        self.tgt_dist = 9999
        self.tgt_steer = 9999
        self.goal_dist = 0.20
        self.la_dist = 0.25
        self.collision_dist = 0.141
        self.rB_phase = 0
        self.motion = [[1,0,1],[0,1,1],[-1,0,1],[0,-1,1]]

        self.range_sub_ = rospy.Subscriber("/range_pub", Float32MultiArray, self.rangeCallback, queue_size=1)
        self.odom_sub_ = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        self.x_wall_sub_ = rospy.Subscriber("/x_wall_pub", Int16MultiArray, self.xwallcallback, queue_size=1)
        self.y_wall_sub_ = rospy.Subscriber("/y_wall_pub", Int16MultiArray, self.ywallcallback, queue_size=1)

        self.control_pub_ = rospy.Publisher("/pathplan/control", Point, queue_size=10)
        self.target_pub_ = rospy.Publisher("/pathplan/target", Point, queue_size=10)
        self.curr_pub_ = rospy.Publisher("/pathplan/curr", Point, queue_size=10)
        
        rospy.loginfo("PathPlan Node Intitialized")
        
    def odomCallback(self,msg):
        self.pos_y_ = msg.pose.pose.position.y
        self.pos_x_ = msg.pose.pose.position.x

        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        self.ang_z_ = m.atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_z*q_z + q_y*q_y)); 
        
        self.AstarPlanner()
        rx_size = len(self.rx)
        if rx_size > 1 and (not self.goal_reached):
            self.rx = self.rx[::-1]
            self.ry = self.ry[::-1]
            self.spath = SpiralPath.calcSpiralPath(self.rx, self.ry, 0.05)
            self.calcTargetIndex()

            self.target_x = self.spath[0][self.tgt_idx]
            self.target_y = self.spath[1][self.tgt_idx]
            self.target_heading = self.spath[2][self.tgt_idx]

            self.tgt_dist = m.sqrt(m.pow(self.target_x - self.pos_x_, 2) + m.pow(self.target_y - self.pos_y_, 2))
            self.tgt_steer = self.normalizeAngle(self.target_heading - self.ang_z_)

            rospy.loginfo("--------A* Global Path--------")
            for i in range(0, rx_size):
                rospy.loginfo("X: %.2f, Y: %.2f", self.rx[i], self.ry[i])
            rospy.loginfo("--------Stanley Control---------")
            number = 3
            if number > rx_size:
                number = rx_size
            for i in range(0, number):
                rospy.loginfo("X: %.2f, Y: %.2f", self.spath[0][i], self.spath[1][i])
            rospy.loginfo("------------------------------")
            rospy.loginfo("Target |X:%.2f,Y:%.2f,HDG:%.2f", self.target_x, self.target_y,self.target_heading)
            rospy.loginfo("Robot  |X:%.2f,Y:%.2f,HDG:%.2f", self.pos_x_, self.pos_y_, self.ang_z_)
            rospy.loginfo("Error  |front: %.2f, turn:%.2f", self.tgt_dist, self.tgt_steer)
        else:
            rospy.loginfo("---------Last approach----------")

            self.target_x = GOAL_X+ self.dist_to_center
            self.target_y = GOAL_Y+ self.dist_to_center
            self.target_heading = m.atan2((GOAL_Y+self.dist_to_center) - self.pos_y_ ,  (GOAL_X+self.dist_to_center) - self.pos_x_)

            self.tgt_dist = m.sqrt(m.pow(self.pos_x_ - (GOAL_X + self.dist_to_center),2) + m.pow(self.pos_y_ - (GOAL_Y+ self.dist_to_center),2))
            self.tgt_steer = self.normalizeAngle(m.atan2((GOAL_Y + self.dist_to_center) - self.pos_y_, (GOAL_X + self.dist_to_center) - self.pos_x_) - self.ang_z_)

            
            rospy.loginfo("------------------------------")
            rospy.loginfo("Target |X:%.2f,Y:%.2f,HDG:%.2f", self.target_x, self.target_y,self.target_heading)
            rospy.loginfo("Robot  |X:%.2f,Y:%.2f,HDG:%.2f", self.pos_x_, self.pos_y_, self.ang_z_)
            rospy.loginfo("Error  |front: %.2f, turn:%.2f", self.tgt_dist, self.tgt_steer)

            if abs(self.pos_x_ - (GOAL_X + self.dist_to_center)) < self.goal_dist and abs(self.pos_y_ - (GOAL_Y + self.dist_to_center)) < self.goal_dist:
                self.goal_reached = GOAL_REACH
                self.target_x = 0
                self.target_y = 0
                self.target_heading = 0
                self.tgt_dist = 0
                self.tgt_steer = 0
                rospy.loginfo("****************************")
                rospy.loginfo("Goal Reach!, X:%.2f, Y:%.2f", self.pos_x_, self.pos_y_)
                rospy.loginfo("****************************")
        

        if self.nearwall() and self.rB_phase == 0:
            self.rB_phase = 1
        if self.rB_phase != 0:
            self.recoveryBehaviour()
        
        if abs(self.tgt_steer) > PI/2:
            self.tgt_dist = 0

        control_cmd = Point()
        target_point = Point()
        curr_position = Point()

        control_cmd.x = self.tgt_dist
        control_cmd.y = self.tgt_steer
        control_cmd.z = self.goal_reached

        target_point.x = self.target_x
        target_point.y = self.target_y
        target_point.z = self.target_heading*180/PI

        curr_position.x = self.pos_x_
        curr_position.y = self.pos_y_
        curr_position.z = self.ang_z_*180/PI

        self.control_pub_.publish(control_cmd)
        self.target_pub_.publish(target_point)
        self.curr_pub_.publish(curr_position)

    def rangeCallback(self,msg):
        self.min_ob_dist = msg.data[0]
        self.min_ob_ang = msg.data[1]

    def xwallcallback(self,msg):
        data_size = len(msg.data)
        for i in range(0, data_size):
            j = int(i/(GRID_SIZE+1))
            self.x_wall[ i%(GRID_SIZE+1)][j] = msg.data[i]
    def ywallcallback(self,msg):
        data_size = len(msg.data)
        for i in range(0, data_size):
            j = int(i/(GRID_SIZE))
            self.y_wall[i%GRID_SIZE][j] = msg.data[i]
    

    def AstarPlanner(self):
        self.rx = []
        self.ry = []
        self.open_set = {}
        self.close_set = {}
        position_x = self.pos_x_ - self.dist_to_center
        position_y = self.pos_y_ - self.dist_to_center

        nstart = [self.calcXYIndex(position_x, MIN_X), 
                  self.calcXYIndex(position_y, MIN_Y),
                  0.0,
                  -1]
        self.ngoal = [self.calcXYIndex(GOAL_X, MIN_X),
                 self.calcXYIndex(GOAL_Y, MIN_Y),
                 0.0,
                 -1]

        self.open_set[self.calcGridIndex(nstart)] = nstart
        while True:
            if len(self.open_set) == 0:
                goal_id = int(self.calcMinCostIndex(self.close_set))
                break

            c_id = self.calcMinCostIndex(self.open_set)
            current = self.open_set[c_id]
            
            if current[0] == self.ngoal[0] and current[1] == self.ngoal[1]:
                rospy.loginfo("You have found your path")
                self.ngoal[3] = current[3]
                self.ngoal[2] = current[2]
                break

            self.open_set.pop(c_id)
            self.close_set[c_id] = current

            for i in range(0,4):
                node = [current[0]+ int(self.motion[i][0]),
                        current[1]+ int(self.motion[i][1]),
                        current[2]+ self.motion[i][2],
                        c_id]
                n_id = int(self.calcGridIndex(node))

                if not self.checkCollision(current, node):
                    continue
                if n_id in self.close_set.keys():
                    continue
                if n_id not in self.open_set.keys():
                    self.open_set[n_id] = node
                else:
                    if self.open_set[n_id][2] > node[2]:
                        self.open_set[n_id] = node
        self.calcFinalPath()


    def calcXYIndex(self, position, min_pos):
        return int(round(position-min_pos)/RESOLUTION)

    def calcGridIndex(self,node):
        return int((node[1] - MIN_Y)*GRID_SIZE + (node[0] - MIN_X))

    def calcGridPosition(self, idx, min_pos):
        return idx* RESOLUTION + min_pos

    def calcMinCostIndex(self, node_set):
        set1 = node_set
        min_cost = 9999.0
        min_idx = 9999
        for i in set1:
            cost = set1[i][2] + self.calcHeuristic(self.ngoal, set1[i])
            if cost < min_cost:
                min_cost = cost
                min_idx = i
        return int(min_idx)
    
    def calcHeuristic(self, n1, n2):
        weight = 1.0
        return weight*m.sqrt(m.pow(n1[0]-n2[0],2) + m.pow(n1[1]-n2[1],2))
    
    def checkCollision(self, curr, next):
        px = self.calcGridPosition(next[0],MIN_X)
        py = self.calcGridPosition(next[1],MIN_Y)
        if px >= GRID_SIZE or px < MIN_X:
            return 0
        if py >= GRID_SIZE or py < MIN_Y:
            return 0
        
        dx = int(next[0] - curr[0])
        dy = int(next[1] - curr[1])

        collision = False
        if dx>0 and dy == 0:
            collision = self.x_wall[next[0]][curr[1]]
        if dx<0 and dy == 0:
            collision = self.x_wall[curr[0]][curr[1]]
        if dx == 0 and dy>0:
            collision = self.y_wall[curr[0]][next[1]]
        if dx == 0 and dy<0:
            collision = self.y_wall[curr[0]][curr[1]]

        if collision:
            return 0
        else:
            return 1

    def calcFinalPath(self):
        self.rx.append(self.calcGridPosition(self.ngoal[0], MIN_X) + self.dist_to_center)
        self.ry.append(self.calcGridPosition(self.ngoal[1], MIN_Y) + self.dist_to_center)

        pind = int(self.ngoal[3])
        while pind != -1:
            n = self.close_set[pind]
            self.rx.append(self.calcGridPosition(n[0], MIN_X) + self.dist_to_center)
            self.ry.append(self.calcGridPosition(n[1], MIN_Y) + self.dist_to_center)
            pind = n[3]

    def calcTargetIndex(self):
        min_dist = 9999.0
        for i in range(0, len(self.spath[0])):
            d = m.sqrt(
                m.pow((self.pos_x_ + self.la_dist*m.cos(self.ang_z_) - self.spath[0][i]),2) + 
                m.pow((self.pos_y_ + self.la_dist*m.sin(self.ang_z_) - self.spath[1][i]),2))
            if d < min_dist:
                min_dist = d
                self.tgt_idx = int(i)
    
    def normalizeAngle(self,angle):
        if angle > PI:
            angle -= 2*PI
            return self.normalizeAngle(angle)
        elif angle < -PI:
            angle += 2*PI
            return self.normalizeAngle(angle)
        return angle

    def nearwall(self):
        if (self.min_ob_dist < self.collision_dist):
            return 1
        else:
            return 0

    def recoveryBehaviour(self):
        rospy.loginfo("!!!RECOVERY BEHAVIOUR IS TRIGGERED!!!")
        min_dist = 9999.0
        nearest_idx = -1
        for i in range(0, len(self.rx)):
            d = m.sqrt(m.pow(self.pos_x_ - self.rx[i], 2) + 
                       m.pow(self.pos_y_ - self.ry[i], 2))
            if d < min_dist:
                min_dist = d
                nearest_idx = i
        if self.rB_phase == 1:
            if abs(self.min_ob_ang) < PI/2:
                self.tgt_dist = -.05
                self.tgt_steer = -.1* self.normalizeAngle(m.atan2(self.ry[nearest_idx] - self.pos_y_, self.rx[nearest_idx] - self.pos_x_) - self.ang_z_)
            else:
                self.tgt_dist = .05
                self.tgt_steer = .1* self.normalizeAngle(m.atan2(self.ry[nearest_idx] - self.pos_y_, self.rx[nearest_idx] - self.pos_x_) - self.ang_z_)  
            if abs(min_dist) < 0.3 or self.min_ob_dist > 0.3:
                self.rB_phase = 2
        if self.rB_phase == 2:
            self.tgt_dist = 0.00
            self.tgt_steer = self.normalizeAngle(self.spath[2] - self.ang_z_)
            if abs(self.tgt_steer) < 0.3:
                self.rB_phase = 0
        else:
            return 

            


                
            
            
            
            
                
            

                
            
            
        
        




if __name__=='__main__':
    rospy.init_node('path_plan_node')
    planner = PathPlan()
    rospy.spin()
