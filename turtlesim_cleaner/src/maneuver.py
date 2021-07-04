#!/usr/bin/env python
import rospy
import numpy as np
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Sub:
    def __init__(self):
        self.i = 0
        
        self.target_x = 0
        self.tartet_y = 0
        self.target_theta = 0

        self.turt1_x = 0
        self.turt1_y = 0
        self.turt1_theta = 0
        self.turning_done = 0
        self.done = 0
        self.turn = 0 # 0 => left, 1 => right
        self.vel = 1
        self.pose_val = Pose()
        self.twist = Twist()
        
        self.sub_turt1 = rospy.Subscriber('/turtle1/pose', Pose, self.turt1_pose)
        self.sub_turt2 = rospy.Subscriber('/turtle2/pose', Pose, self.turt2_pose)
        # sub_turt2 = rospy.Subscriber('/turtle2/cmd_vel', Twist, self.turt2_vel)
        self.pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    def turt1_pose(self,msg):
        self.turt1_x = msg.x
        self.turt1_y = msg.y
        if self.i == 0:
            self.target_x = msg.x
            self.target_y = msg.y
            self.target_theta = msg.theta
            self.i = 1
            # print("x" + str(self.target_x))
            # print(self.target_y)
            
    def turt2_pose(self, msg):
        self.pose_val.x = msg.x
        self.pose_val.y = msg.y
        self.pose_val.theta = msg.theta
        self.pose_val.linear_velocity = msg.linear_velocity
        self.pose_val.angular_velocity = msg.angular_velocity
        if self.i == 1:
            self.target_theta = np.arctan2(self.target_y-self.pose_val.y,self.target_x-self.pose_val.x)
            self.i = 2
        
        if self.turning_done == 1:
            dist = np.linalg.norm((self.turt1_x - self.pose_val.x,self.turt1_y - self.pose_val.y))
            if dist < 2:
                theta_diff = np.arctan2(self.turt1_y-self.pose_val.y, self.turt1_y-self.pose_val.x)
                if theta_diff < 0:
                    # turn leftwards
                    self.turn = 0
                    self.twist.linear.x = self.vel
                    if (self.pose_val.theta - self.target_theta) < np.pi/2 and self.done == 0:
                        self.twist.angular.z = 2*self.vel
                    else:
                        if self.done != 2:
                            self.done = 1
                        self.twist.angular.z = -2*self.vel
                        if (self.pose_val.theta - self.target_theta < 0) or self.done == 2:
                            self.done = 2
                            self.twist.angular.z = 0
                            self.twist.linear.x = 0.2*self.twist.linear.x
                else:
                    # turn rightwards
                    self.turn = 1
                    self.twist.linear.x = self.vel
                    if (self.pose_val.theta - self.target_theta) > -np.pi/2 and self.done == 0:
                        self.twist.angular.z = -2*self.vel
                    else:
                        if self.done != 2:
                            self.done = 1
                        self.twist.angular.z = 2*self.vel
                        if (self.pose_val.theta - self.target_theta > 0) or self.done == 2:
                            self.done = 2
                            self.twist.angular.z = 0
                            self.twist.linear.x = 0.2*self.twist.linear.x
                self.pub.publish(self.twist)
                

            else:
                self.twist.linear.x = self.vel
                self.twist.angular.z = 0
                if self.done == 1:
                    if self.turn == 0:
                        self.twist.linear.x = self.vel
                        if (self.pose_val.theta - self.target_theta) < np.pi/2 and self.done == 0:
                            self.twist.angular.z = 2*self.vel
                        else:
                            if self.done != 2:
                                self.done = 1
                            self.twist.angular.z = -2*self.vel
                            if (self.pose_val.theta - self.target_theta < 0) or self.done == 2:
                                self.done = 2
                                self.twist.angular.z = 0
                                self.twist.linear.x = 0.2*self.twist.linear.x
                    else:
                        self.twist.linear.x = self.vel
                        if (self.pose_val.theta - self.target_theta) > -np.pi/2 and self.done == 0:
                            self.twist.angular.z = -2*self.vel
                        else:
                            if self.done != 2:
                                self.done = 1
                            self.twist.angular.z = 2*self.vel
                            if (self.pose_val.theta - self.target_theta > 0) or self.done == 2:
                                self.done = 2
                                self.twist.angular.z = 0
                                self.twist.linear.x = 0.2*self.twist.linear.x

                if self.done == 2:
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0
                self.pub.publish(self.twist)
        else:
            theta_diff = self.target_theta - self.pose_val.theta
            if theta_diff > 0.05:
                self.twist.angular.z = 0.3*(self.target_theta - self.pose_val.theta)
                self.twist.linear.x = 0
                self.pub.publish(self.twist)
            else:
                self.turning_done = 1
if __name__ == "__main__":
    rospy.init_node('A4')
    sub = Sub()
    rospy.spin()

