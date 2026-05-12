#!/usr/bin/env python3
# FSM manager

import rclpy 
import time
from rclpy.node import Node


from example_interfaces.msg import String 
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry 
from ele434_team15_2026.msg import KeyInfo #import key info as a new type

from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan  #Laser scan data type
import numpy as np #Used for data put togethers

from ele434_team15_2026_modules.tb3_tools import quaternion_to_euler 
from math import sqrt, pow, pi, dist

class VelocityControl(Node): 

    def __init__(self):
        super().__init__("velocity_control") 

        #Tunning values maximum
        self.linear_vel=0.2 #straight velocity
        self.angluar_vel=-0.5 #angular velcoity
    
        #Set up key_info
          #Create key info to be published
        self.key_info=KeyInfo()
        self.key_info.state= "Waypoint"
        self.key_info.waypoint_x=0
        self.key_info.waypoint_y=0
        self.key_info.vel_trigger="False"
        self.accelerated=True

        self.timestamp = self.get_clock().now().nanoseconds

    



        self.my_publisher = self.create_publisher( #publisher to velocity
            msg_type=TwistStamped,
            topic="/cmd_vel",
            qos_profile=10,
        ) 

        self.key_info_subscriber = self.create_subscription(
            msg_type=KeyInfo, 
            topic="/key_info", 
            callback=self.key_info_callback, 
            qos_profile=10,
        )

        publish_rate = 10 # Hz
        self.timer = self.create_timer(
            timer_period_sec=1/publish_rate, 
            callback=self.timer_callback
        ) 
        self.shutdown = False #to sort out CTRL+C

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised." 
        )

    def key_info_callback(self, key_info_message: KeyInfo): 
        # recieve key info data
        self.key_info=key_info_message

        self.get_logger().info( 
            f"state: {self.key_info.state}",
            throttle_duration_sec=1, 
        ) 

    def timer_callback(self): 

        #Create velocity to be published
        topic_msg = TwistStamped()
        topic_msg.twist.linear.x = 0.0
        topic_msg.twist.linear.y = 0.0
        topic_msg.twist.linear.z = 0.0
        topic_msg.twist.angular.x = 0.0
        topic_msg.twist.angular.y = 0.0
        topic_msg.twist.angular.z = 0.0
        time_now = self.get_clock().now().nanoseconds
        elapsed_time = (time_now - self.timestamp) * 1e-9
        previous_state="none"

        if self.key_info.state != previous_state: #rest timer
            timestamp=time_now



        if self.key_info.vel_trigger == "Linear":
            if elapsed_time > 2:
                topic_msg.twist.linear.x = 0.25
            if elapsed_time > 1.5:
                topic_msg.twist.linear.x = 0.2
            elif elapsed_time > 1:
                topic_msg.twist.linear.x = 0.15
            elif elapsed_time > 0.5:
                topic_msg.twist.linear.x = 0.1
            elif elapsed_time > 0:
                topic_msg.twist.linear.x = 0.05
            self.get_logger().info( f"Linear Speed increase time, speed{topic_msg.twist.linear.x}",throttle_duration_sec=1, ) 
        elif self.key_info.vel_trigger == "Angular":
            if elapsed_time > 3.0:
                topic_msg.twist.angular.z = 1.2
            elif elapsed_time > 2.5:
                topic_msg.twist.angular.z = 1.0
            elif elapsed_time > 2.0:
                topic_msg.twist.angular.z = 0.8
            elif elapsed_time > 1.0:
                topic_msg.twist.angular.z = 0.6
            elif elapsed_time > 0.5:
                topic_msg.twist.angular.z = 0.4
            elif elapsed_time > 0.0:
                topic_msg.twist.angular.z = 0.2
            self.get_logger().info( f"Angluar Speed increase time, speed:{topic_msg.twist.angular.z}",throttle_duration_sec=1, ) 
         elif self.key_info.vel_trigger == "Angular2":
            if elapsed_time > 3.0:
                topic_msg.twist.angular.z = -1.2
            elif elapsed_time > 2.5:
                topic_msg.twist.angular.z = -1.0
            elif elapsed_time > 2.0:
                topic_msg.twist.angular.z = -0.8
            elif elapsed_time > 1.0:
                topic_msg.twist.angular.z = -0.6
            elif elapsed_time > 0.5:
                topic_msg.twist.angular.z = -0.4
            elif elapsed_time > 0.0:
                topic_msg.twist.angular.z = -0.2
            self.get_logger().info( f"Angluar Speed increase time, speed:{topic_msg.twist.angular.z}",throttle_duration_sec=1, ) 
        elif self.key_info.vel_trigger == "Stop":
            self.get_logger().info( f"Angluar Speed increase time",throttle_duration_sec=1, ) 
        previous_state=self.key_info.state

        self.my_publisher.publish(topic_msg)
    
       

    def on_shutdown(self):
        self.get_logger().info(
            "Stopping the robot..."
        )
        self.my_publisher.publish(TwistStamped()) #sets all velocities to 0
        self.shutdown = True #Starts the flag

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO #Option to handle signals ourselves
    ) 
    velocity_control = VelocityControl()
    try: #Use try except to be able to more easily interupt it
        rclpy.spin(velocity_control) 
    except KeyboardInterrupt: 
        print(
            f"{velocity_control.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally: 
        velocity_control.on_shutdown() #Calll shutdown function
        while not velocity_control.shutdown:  #This will wait untill flag has been set, i.e function is done shutting down
            continue
        velocity_control.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__': 
    main()