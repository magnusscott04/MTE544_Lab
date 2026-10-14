# Imports
import rclpy
from rclpy.node import Node
from utilities import Logger, euler_from_quaternion

# Import QOS Keywords
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Import MSG Libraries
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

#Set QOS Size to 10 (Standard from ROS Docs)
qos_size = 10

CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        #Initialize variables for motion functions
        self.radius_=0.0
        self.spiral_t = 0
        self.acc_t = 0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # Create publisher with name Twist publishing to /cmd_vel topic with QOS size matching subscribers
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', qos_size)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["angle_increment", "stamp", "ranges"])
        
        # Setup QOS profiles for subscribers - Matched publishers during lab session
        qosR=QoSProfile(depth = qos_size, reliability = ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.VOLATILE, history = HistoryPolicy.KEEP_LAST)
        qosBE=QoSProfile(depth = qos_size, reliability = ReliabilityPolicy.BEST_EFFORT, durability = DurabilityPolicy.VOLATILE, history = HistoryPolicy.KEEP_LAST)
        
        #IMU Subscriber
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qosBE
        )
    
        # ENCODER subscription
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qosBE
        )
        
        # LaserScan subscription 
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qosBE
        )
        
        self.create_timer(0.1, self.timer_callback)


    # IMU Callback Function
    def imu_callback(self, imu_msg: Imu):
        self.imu_initialized = True #Set flag to true for validation

        #Extract Acceleration Data
        acc_x = imu_msg.linear_acceleration.x
        acc_y = imu_msg.linear_acceleration.y
        angular_z = imu_msg.angular_velocity.z
        
        #Extract time stamp
        stamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        self.imu_logger.log_values([acc_x, acc_y, angular_z, stamp]) #Log list of values with log_values func

    # Odometry Callback Function
    def odom_callback(self, odom_msg: Odometry):
        self.odom_initialized = True  #Set flag to true for validation
        
        # Extract Position Data
        x= odom_msg.pose.pose.position.x
        y= odom_msg.pose.pose.position.y

        # Extract Quaternion Data
        quat = odom_msg.pose.pose.orientation
        yaw = euler_from_quaternion(quat)
        
        #Extract time stamp
        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        
        self.odom_logger.log_values([x,y,yaw, stamp]) #Log list of values with log_values func
                
    # Laser Callback Function
    def laser_callback(self, laser_msg: LaserScan):
        self.laser_initialized = True  #Set flag to true for validation

        #Extract Distance Values + Angle between measurements
        ranges = list(laser_msg.ranges)
        angle_increment = laser_msg.angle_increment                
        
        #Extract time stamp
        stamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
                
        self.laser_logger.log_values([angle_increment,stamp, *ranges]) #Log list of values with log_values func
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            print(f"Waiting for sensors... Odom: {self.odom_initialized}, Laser: 	{self.laser_initialized}, IMU: {self.imu_initialized}")
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    # Circle - Constant Linear & Angular Velocity
    def make_circular_twist(self):
        msg=Twist() # Store Twist Publisher in MSG
        
        #Set command velocities
        msg.linear.x = 0.2 # 0.2 m/s forwards
        msg.angular.z = 0.4 # 0.4 radians per second of rotation about z
        return msg

    # Spiral - Time varying linear velocity, constant angular velocity
    def make_spiral_twist(self):
        msg=Twist()#Store Twist Publisher in MSG
        
        #Set command velocities
        w = 0.5 # Constant angular velocity
        self.spiral_t +=1 #Inc spiral stamp val
        msg.angular.z = w #Assign constant velocity
        msg.linear.x = min(0.001*self.spiral_t,0.3) #Either increase linear velocity, or hold value
        return msg
    
    # Accelerating Line - Time varying linear velocity, zero angular velociy
    def make_acc_line_twist(self):
        msg=Twist() #Store Twist Publisher in MSG
        
        # Set command velocities
        self.acc_t += 0.01 #Increment time stamp
        msg.linear.x = min(self.acc_t,0.3) #Either increase linear velocity, or hold value
        msg.angular.z = 0.0 # Set angular velocity to 0
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
