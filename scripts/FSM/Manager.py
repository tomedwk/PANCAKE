#!/usr/bin/env python3
# FSM manager

import rclpy 
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

class BasicObstacle(Node): 

    def __init__(self):
        super().__init__("basic_obstacle") 

        #Tunning values
        self.linear_vel=0.1 #straight velocity
        self.angluar_vel=0.2 #angular velcoity
        self.collision_zone_left= 19 #edge of collision zone left
        self.collision_zone_right=-18 #collision zone right
        self.distance_collision=0.4 #distance before object needs to be avoided
        self.turn_done=False #to know whether turn is done
        
        self.collision_min=float("nan")
        self.theta_z = 0.0
        self.theta_zref = 0.0
        self.y_ref=0.0
        self.x_ref=0.0
        self.y=0.0
        self.x=0.0
        self.first_message = False #Whether it has been the first message

        #Set up key_info
          #Create key info to be published
        self.key_info=KeyInfo()
        self.key_info.state= "Waypoint"
        self.key_info.waypoint_x=0
        self.key_info.waypoint_y=0

        self.distance=0




        self.my_publisher = self.create_publisher( #publisher to velocity
            msg_type=TwistStamped,
            topic="/cmd_vel",
            qos_profile=10,
        ) 

        self.my_publisher_key= self.create_publisher( #publisher to KeyInfo
            msg_type=KeyInfo,
            topic="/key_info",
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
        #set pose to the right thing
        self.y=pose.position.y
        self.x=pose.position.x

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
        valid_data = collision_zone[collision_zone != float("inf")] #filter out infinite ones (sim) 
        valid_data = collision_zone[collision_zone != 0] #filter out zeros (real robot)

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
        if self.key_info.state == "Waypoint":
            if not np.isnan(self.collision_min):
                if self.collision_min < self.distance_collision:
                    self.get_logger().info("Obstacle detected")
                    self.key_info.state ="Obstacle"

        elif self.key_info.state == "Obstacle":
            diff = self.theta_z - self.theta_zref
            diff = (diff + pi) % (2 * pi) - pi  # Wrap to [-180°, 180°]
            if abs(diff) >= (40*pi/180):
                self.theta_zref=self.theta_z
                self.get_logger().info(f"time to go straight, distance: {self.distance}")
                self.turn_done=True
                self.x_ref=self.x
                self.y_ref=self.y
            if self.distance > 0.1 and self.turn_done == True:
                self.get_logger().info(f"go straight done")
                self.x_ref=self.x
                self.y_ref=self.y
                self.turn_done=False
                self.key_info.state ="Waypoint"

            



        #current state
        if self.key_info.state == "Waypoint":
            #Do waypoint bit
            self.get_logger().info(
            f"State: {self.key_info.state} collision_min {self.collision_min:.2}",
            throttle_duration_sec = 1,
        ) #printing but only every 1 second
        elif self.key_info.state == "Obstacle":
            #Do obstacle bit
            #Turn 45 degrees
            if self.turn_done== False:
                topic_msg.twist.linear.x=0.0
                topic_msg.twist.angular.z=self.angluar_vel
                self.get_logger().info(f"turning  yaw:{(self.theta_z - self.theta_zref)*180/pi:.3}",
                throttle_duration_sec = 2,
                )
            elif self.turn_done ==True:
                self.distance=dist([self.x_ref, self.y_ref],[self.x, self.y])
                self.get_logger().info(f"time to go straight, distance: {self.distance} turnDone : {self.turn_done}")
                topic_msg.twist.linear.x=self.linear_vel #go straight

            self.my_publisher.publish(topic_msg)
    

        
        self.my_publisher_key.publish(self.key_info)
       

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