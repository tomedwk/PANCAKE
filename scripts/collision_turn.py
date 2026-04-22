#!/usr/bin/env python3
# ROS 2 Code to detect obstacle and avoid

import rclpy 
from rclpy.node import Node


from example_interfaces.msg import String 
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry 

from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan  #Laser scan data type
import numpy as np #Used for data put togethers

from part2_navigation_modules.tb3_tools import quaternion_to_euler 
from math import sqrt, pow, pi 

class BasicObstacle(Node): 

    def __init__(self):
        super().__init__("basic_obstacle") 

        #Tunning values
        self.linear_vel=0.2 #straight velocity
        self.angluar_vel=0.2 #angular velcoity
        self.collision_zone_left= 19 #edge of collision zone left
        self.collision_zone_right=-18 #collision zone right
        self.distance_collision=0.4 #distance before object needs to be avoided
        
        self.obstacle=False
        self.collision_min=float("nan")
        self.theta_z = 0.0
        self.theta_zref = 0.0
        self.first_message = False #Whether it has been the first message
        self.turn=False #Whether you need to turn

        self.my_publisher = self.create_publisher( #publisher to velocity
            msg_type=TwistStamped,
            topic="/cmd_vel",
            qos_profile=10,
        ) 
        self.lidar_sub = self.create_subscription( #Subscription to LIDAR
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        ) 
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
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

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 

        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation) 
        self.theta_z = yaw 

        if not self.first_message: 
            self.first_message = True
            self.theta_zref = self.theta_z


    def lidar_callback(self, scan_data: LaserScan):
        #Constructing collision zone (Tume these)
        collision_left = scan_data.ranges[0:self.collision_zone_left] 
        collision_right = scan_data.ranges[self.collision_zone_right:] 
        collision_zone = np.array(collision_right + collision_left) #Convert to array rather then list
        valid_data = collision_zone[collision_zone != float("inf")] #filter out infinite ones

        if np.shape(valid_data)[0] > 0: 
            self.collision_min=valid_data.min()
        else:
            self.collision_min= float("nan")

    def timer_callback(self): 

       #Create velocity to be published
        topic_msg = TwistStamped()
        topic_msg.twist.linear.x = 0.0
        topic_msg.twist.linear.y = 0.0
        topic_msg.twist.linear.z = 0.0
        topic_msg.twist.angular.x = 0.0
        topic_msg.twist.angular.y = 0.0
        topic_msg.twist.angular.z = 0.0

        
        #does state need to be changed
        if self.obstacle== False:
            if not np.isnan(self.collision_min):
                if self.collision_min < self.distance_collision:
                    self.get_logger().info("Obstacle detected")
                    self.obstacle=True

        elif self.obstacle== True:
            diff = self.theta_z - self.theta_zref
            diff = (diff + pi) % (2 * pi) - pi  # Wrap to [-180°, 180°]
            if (diff) >= (45*pi/180):
                self.theta_zref=self.theta_z
                self.get_logger().info(f"time to go straight")
                self.obstacle=False



        #current state
        if self.obstacle==False:
            topic_msg.twist.linear.x=self.linear_vel #go straight
            self.get_logger().info(
            f"State: {self.obstacle} collision_min {self.collision_min:.2}",
            throttle_duration_sec = 1,
        ) #printing but only every 1 second
        elif self.obstacle== True:
            #Turn 45 degrees
            topic_msg.twist.linear.x=0.0
            topic_msg.twist.angular.z=self.angluar_vel
            self.get_logger().info(f"turning  yaw:{(self.theta_z - self.theta_zref)*180/pi:.3}",
            throttle_duration_sec = 2,
            )
    

        

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
    basic_obstacle = BasicObstacle()
    try: #Use try except to be able to more easily interupt it
        rclpy.spin(basic_obstacle) 
    except KeyboardInterrupt: 
        print(
            f"{basic_obstacle.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally: 
        basic_obstacle.on_shutdown() #Calll shutdown function
        while not basic_obstacle.shutdown:  #This will wait untill flag has been set, i.e function is done shutting down
            continue
        basic_obstacle.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__': 
    main()