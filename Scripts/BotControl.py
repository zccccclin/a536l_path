#! /usr/bin/python
import rospy
from geometry_msgs.msg import Twist 
from geometry_msg.msg import Point
from PreDefine import *

class BotControl:
    def __init__(self):
        self.GOAL_NOT_REACH = GOAL_NOT_REACH
        self.GOAL_REACH = GOAL_REACH
        self.target_x_ = 0
        self.target_y_ =0
        self.goal_reached_ = 0
        self.error_pos_ = 0
        self.error_pos_prev_ = 0
        self.error_heading_ = 0
        self.error_heading_prev_ = 0
        self.dt = 0.2
        self.max_vel = 0.4
        self.max_ang = PI
        self.target_sub_ = rospy.Subscriber('/pathplan/control', Point, self.targetCallBack, queue_size=1)
        self.control_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.loginfo('BotControl Node Initialized')

    def targetCallBack(self,msg):
        self.target_x_ = msg.x
        self.target_y_ = msg.y
        self.goal_reached_ = msg.z
        self.controlPub()
    
    def controlPub(self):
        trans_x = 0
        trans_heading = 0
        self.error_pos_prev_ = self. error_pos_
        self.error_heading_prev_ = self.error_heading_
        
        if self.error_heading_ < PI:
            self.error_heading_ += 2*PI
        if self.error_heading_ > PI:
            self.error_heading_ -= 2*PI
        
        #PID controller algo
        I_heading = self.dtdt*self.error_heading_
        I_pos = self.dt*self.error_pos_
        D_heading = self.error_heading_prev_ - self.error_heading_
        D_pos = self.error_pos_prev_ - self.error_pos_

        trans_x = Kp_x * self.error_pos_ + Ki_x * I_pos + Kd_x * D_pos
        trans_heading = Kp_a * self.error_heading_ + Ki_a * I_heading + Kd_a * D_heading; 

        if trans_x > self.max_vel:
            trans_x = self.max_vel
        if trans_x < -self.max_vel:
            trans_x = -self.max_vel
        if trans_heading > self.max_ang:
            trans_heading = self.max_ang
        if trans_heading < -self.max_ang:
            trans_heading = -self.max_ang

        cmd = Twist()
        cmd.linear.x = trans_x
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = trans_heading

        self.control_pub_.publish(cmd)
        rospy.loginfo('error_x: %f, error_heading: %f.', self.target_x_, self.target_y_)
        rospy.loginfo('trans_x: %f, trans_ang: %f.', trans_x, trans_heading)


if __name__=='__main__':
    rospy.init_node('bot_control_node')
    controller = BotControl()
    rospy.spin()

