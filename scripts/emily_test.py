#!/usr/bin/env python3
# ROS 2 Code to detect obstacle and avoid

import rclpy 
from rclpy.node import Node


from example_interfaces.msg import String 
from geometry_msgs.msg import TwistStamped
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan  #Laser scan data type
import numpy as np #Used for data put togethers

class BasicObstacle(Node): 

    def __init__(self):
        super().__init__("basic_obstacle") 

        self.obstacle=False
        self.collision_min=float("nan")

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

        publish_rate = 10 # Hz
        self.timer = self.create_timer(
            timer_period_sec=1/publish_rate, 
            callback=self.timer_callback
        ) 
        self.shutdown = False #to sort out CTRL+C

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised." 
        )

    def lidar_callback(self, scan_data: LaserScan):
        #Constructing collision zone
        collision_left = scan_data.ranges[0:19] 
        collision_right = scan_data.ranges[-18:] 
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
                if self.collision_min < 0.5:
                    self.get_logger().info("Obstacle detected")
                    self.obstacle=True

        #elif self.obstacle== True:
            #check if change state needs to happen



        #current state
        if self.obstacle==False:
            topic_msg.twist.linear.x=0.2 #go straight
        elif self.obstacle== True:
            topic_msg.twist.linear.x=0.0 #stop

        self.get_logger().info(
            f"State: {self.obstacle} collision_min {self.collision_min}",
            throttle_duration_sec = 1,
        ) #printing but only every 1 second


        

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