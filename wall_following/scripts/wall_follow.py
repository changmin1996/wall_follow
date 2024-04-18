#! /usr/bin/env python3

# for the basic ros
import rospy

# for msgs type
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# for math thing
import numpy as np

# set the max min velocity for angular control
MAX_ANGULAR = 1.0
MIN_ANGULAR = -1.0

# set the Max and Min Linear velocity
MAX_LINEAR = 0.3
MIN_LINEAR = 0.08

"""
this Application get Laser scan data which is on the right side
it convert to cartesian coordinate
use the least square to get linear equation
y = mx + c

for contorl it use PI Control
P_Gain for slop which is "m"
I_Gain for closest distance to (0.0) which is "d"
calculate the control data for angular velocity

publish cmd_vel
"""

class WallFollow:
    def __init__(self):
        # set subsciber & publisher
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laser_callback)
        self.cmd_pub_ = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # flag for initial subscribe
        self.init_flag = False

        # set variable for PI Control
        self.P_gain = 2.0
        self.P_gain_dis = 2.0
        self.step_time = 0.1
        self.object_dis = 0.2 # 20cm
        self.i_control = 0.0

        # set control initial data
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

    def laser_callback(self, msg):
        # set lidar property at the first time of callback 
        if not self.init_flag:
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            self.init_flag = True
        
        # set empty numpy array for x, y
        x_data = np.array([])
        y_data = np.array([])

        # convert to catesian coordinate
        for index, data in enumerate(msg.ranges):
            current_angle = self.angle_min + self.angle_increment * index
            cx = data * np.cos(current_angle)
            cy = data * np.sin(current_angle)
            
            # set ROI and append to array
            if 0.0 < cx < 0.3 and cy > 0.0:
                x_data = np.append(x_data, cx)
                y_data = np.append(y_data, cy)
        
        # make matrix
        A_ = np.vstack([x_data, np.ones(len(x_data))]).T

        # least square to get m, c
        m_, c_ = np.linalg.lstsq(A_, y_data, rcond=None)[0]

        # cal closest distance to (0,0)
        d_ = abs(c_) / (np.sqrt(m_**2 + 1))

        # calculate the control data with PI Control
        control_ = self.PIControl(m_, d_)

        # cal cmd vel
        self.cal_cmd_vel(control_)

        
    def PIControl(self, m, d):
        # cal P control and I contorl
        p_control_ = self.P_gain * m
        p_control_dis =  self.P_gain_dis * (self.object_dis - d)

        
        control_ = p_control_ - p_control_dis
        
        return control_

    def cal_cmd_vel(self, control):
        # the ratio of the linear velocity and angular velocity it should slow down at curve
        linear_angular_ratio = MAX_LINEAR / MAX_ANGULAR

        # cal cmd_vel
        self.cmd.linear.x = MAX_LINEAR - abs(control) * linear_angular_ratio
        self.cmd.linear.x = max(self.cmd.linear.x, MIN_LINEAR)
        self.cmd.angular.z = max(min(MAX_ANGULAR, control), MIN_ANGULAR)


    def control(self): # publish cmd_vel
        self.cmd_pub_.publish(self.cmd)

def main():
    # initialize 
    rospy.init_node('wall_follow')
    wf = WallFollow()
    rate = rospy.Rate(10)
    
    # loop for control (10HZ)
    while not rospy.is_shutdown():
        wf.control()
        rate.sleep()

if __name__ == '__main__':
    main()