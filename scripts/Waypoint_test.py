#!/usr/bin/env python3
# attempt at moving between waypoint 

import rclpy 
from rclpy.node import Node
import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class MoveWaypoint(Node): 

    def __init__(self):
        super().__init__("move_waypoint") 

        self.position = [0,0]
        self.yaw = 0
        
        self.waypoints = [ [0,0], [0,-1], [-1,-1] ]
        self.waypoint_i = 0

        self.counter = 0

        
        self.my_subscriber = self.create_subscription(
            msg_type=Odometry, 
            topic="/odom", 
            callback=self.pose_callback, 
            qos_profile=10,
        )
        
        self.vel_publisher = self.create_publisher(
            msg_type=TwistStamped,
            topic="cmd_vel",
            qos_profile=10,
        ) 

        publish_rate = 10 # Hz
        self.timer = self.create_timer(
            timer_period_sec=1/publish_rate, 
            callback=self.timer_callback
        ) 

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised." 
        )

    def timer_callback(self):
        linear_vel = 0.0
        angular_vel = 0.0

        curr_waypoint = self.waypoints[self.waypoint_i]
        
        target_vect = [ curr_waypoint[0] - self.position[0], curr_waypoint[1] - self.position[1]]
        target_dist = math.sqrt( target_vect[0]**2 + target_vect[1]**2 )
        target_ang = math.atan2(target_vect[1],target_vect[0])

        if target_dist > 0.05:
            
            if (self.yaw - target_ang > 0.1):
                angular_vel = -0.3
            elif (self.yaw - target_ang - 0.1):
                angular_vel = 0.3
            else: 
                linear_vel = 0.1
        else:
            self.waypoint_i += 1

            if self.waypoint_i > len(self.waypoints):
                self.waypoint_i = 0
        
        self.publish_vel(linear_vel, angular_vel)

    
    def pose_callback(self, topic_message: Odometry): 
        pose = topic_message.pose.pose 
    
        pos_x = pose.position.x
        pos_y = pose.position.y

        self.position = [pos_x, pos_y]

        self.yaw = self.quaternion_to_euler(pose.orientation) 

        if self.counter > 10: 
            self.counter = 0
            self.get_logger().info(
                f"x = {pos_x:.3f} (m), y = {pos_y:.3f} (m), yaw = {self.yaw:.3f} (radians)"
            ) 
        else:
            self.counter += 1


    def publish_vel(self, linear_velocity, angular_velocity):
        topic_msg = TwistStamped() 
        topic_msg.twist.linear.x = linear_velocity
        topic_msg.twist.angular.z = angular_velocity
        self.vel_publisher.publish(topic_msg) 

        self.get_logger().info( 
            f"Linear Velocity: {topic_msg.twist.linear.x:.2f} [m/s], "
            f"Angular Velocity: {topic_msg.twist.angular.z:.2f} [rad/s].",
            throttle_duration_sec=1, 
        )

    
    def quaternion_to_euler(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        yaw = math.atan2( 2*(x*z + x*y), 1-2*(y**2 + z**2) )

        return yaw



def main(args=None): 
    rclpy.init(args=args)
    waypoint_follower = MoveWaypoint()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
