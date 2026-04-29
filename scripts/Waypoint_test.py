#!/usr/bin/env python3
'''
Code for moving between a set list of waypoints   
'''

import rclpy 
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class MoveWaypoint(Node): 

    def __init__(self):
        super().__init__("move_waypoint") 

        # initialise robot position
        self.init_position = [0,0]
        self.init_yaw = 0

        self.abs_position = [0,0]
        self.abs_yaw = 0

        # initialise list of waypoints [x,y] & waypoint pointer
        self.waypoints = [ [0,0], [1,0], [1,1], [0,1] ]
        self.waypoint_ptr = 0


        # subscriber to robot odom data
        self.odom_subscriber = self.create_subscription(
            msg_type=Odometry, 
            topic="/odom", 
            callback=self.pose_callback, 
            qos_profile=10,
        )
        
        # pulisher to robot velocity commands
        self.vel_publisher = self.create_publisher(
            msg_type=TwistStamped,
            topic="cmd_vel",
            qos_profile=10,
        ) 
        

        # timer for main control loop
        publish_rate = 10 # Hz
        self.timer = self.create_timer(
            timer_period_sec=1/publish_rate, 
            callback=self.timer_callback
        ) 

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised." 
        )


    def timer_callback(self):
        # control loop: calculate cmd velocities to follow waypoint list

        # initialise velocities as 0
        linear_vel = 0.0
        angular_vel = 0.0

        # calculate relative position 
        rel_position = self.abs_position - self.init_position
        rel_yaw = self.abs_yaw - self.init_yaw


        # get the current waypoint to go towards
        curr_waypoint = self.waypoints[self.waypoint_ptr]
        forward_vect = [math.cos(rel_yaw), math.sin(rel_yaw)]            # unit vector pointing in robot forward direction 

        # vector pointing from robot to target waypoint
        target_vect = [ curr_waypoint[0] - rel_position[0], curr_waypoint[1] - rel_position[1]]

        # distance from robot to target way point
        dist_error = math.sqrt( target_vect[0]**2 + target_vect[1]**2 )

        # signed angle from robot forward direction to target waypoint vector
        angle_error = math.atan2( forward_vect[0]*target_vect[1] - forward_vect[1]*target_vect[0],
                                forward_vect[0]*target_vect[0] + forward_vect[1]*target_vect[1] )
      

        if dist_error > 0.01:
            # robot not at way point => move towards waypoint
            if angle_error > 0.05:
                angular_vel = 0.3

            elif angle_error < -0.05:
                angular_vel = -0.3

            else: 
                linear_vel = 0.1

        else:
            # robot at waypoint => increment waypoint pointer 
            self.waypoint_ptr += 1
            
            # loop pointer back to start if out of range
            if self.waypoint_ptr > len(self.waypoints) - 1:
                self.waypoint_ptr = 0
        
        self.publish_vel(linear_vel, angular_vel)

    
    def pose_callback(self, topic_message: Odometry): 
        # recieve robot odom data
        pose = topic_message.pose.pose 
    
        pos_x = pose.position.x
        pos_y = pose.position.y

        self.abs_position = [pos_x, pos_y]
        self.abs_yaw = self.quaternion_to_euler(pose.orientation) 

        self.get_logger().info( 
            f"x:{pos_x:.3f}, y:{pos_y:.3f}, yaw:{self.yaw:.3f}",
            throttle_duration_sec=1, 
        ) 


    def publish_vel(self, linear_velocity, angular_velocity):
        # publish cmd velocities 
        topic_msg = TwistStamped() 
        topic_msg.twist.linear.x = linear_velocity
        topic_msg.twist.angular.z = angular_velocity
        self.vel_publisher.publish(topic_msg) 

        self.get_logger().info( 
            f"Lin_Vel: {topic_msg.twist.linear.x:.2f}, "
            f"Ang_Vel: {topic_msg.twist.angular.z:.2f}",
            throttle_duration_sec=1, 
        )

    def quaternion_to_euler(self, orientation):
        # convert quaternion rotation to yaw angele
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return yaw

def zero_location(self):
    self.init_position = self.abs_position
    self.init_yaw = self.abs_yaw
    


def main(args=None): 
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    waypoint_follower = MoveWaypoint()
    waypoint_follower.zero_location()
    try:
        rclpy.spin(waypoint_follower)
    except KeyboardInterrupt:
        print("Shutdown request (Ctrl+C) detected...")
    finally:
        waypoint_follower.publish_vel(0.0,0.0)
        waypoint_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
