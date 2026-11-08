import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)
        
        x= np.zeros(6) # Initial state (6x1) set to 0s
        
        Q= 0.5*np.eye(6) # Initialize model noise - 0.5 times 6x6 identity matrix (As instructed)

        R= 0.5*np.eye(4) # Initialize sensor noise - 0.5 times 4x4 identity matrix (As instructed)
        
        P= 0.1*np.eye(6) # initial covariance
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos) #Create odometry subscriber
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)    #Create IMU subscriber
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        
        #z - Measurement vector used for correction stage of kalman filter
        z= np.array([odom_msg.twist.twist.linear.x, #Store odometer measured linear vel
            odom_msg.twist.twist.angular.z,         #Store odometer measured angular vel
            imu_msg.linear_acceleration.x,          #Store IMU measured linear acceleration
            imu_msg.linear_acceleration.y])         #Store IMU measured angular acceleration
        
        # Implement the two steps for estimation
        self.kf.predict() #Predict updates state vector based on the motion model
        self.kf.update(z) #Update corrects prediction based on sensor feedback/confidence
        
        # Get the estimate [x,y,theta,x,v,v_dot]
        xhat=self.kf.get_states()
        
        # Store time stamp
        stamp = odom_msg.header.stamp
        
        # Update the pose estimate to be returned by getPose
        # odom pose [x,y,theta,stamp]
        self.pose=[xhat[0], xhat[1], xhat[2],stamp]

        # Part 4: log your data
        self.loc_logger.log_values([
            imu_msg.linear_acceleration.x,   # Sensor acceleration in x
            imu_msg.linear_acceleration.y,   # Sensor acceleration in y
            xhat[5],                         # KF acceleration in x
            xhat[4]*xhat[3],                 # KF acceleration in y
            xhat[4],                         # KF linear velocity
            xhat[3],                         # KF angular velocity
            xhat[0],                         # KF x position
            xhat[1],                         # KF y position
            Time.from_msg(stamp).nanoseconds # Time stamp
        ])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
