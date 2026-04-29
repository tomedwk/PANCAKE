#!/usr/bin/env python3
# Code to set lidar detected objects as repulsors


import rclpy


from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from example_interfaces.msg import String


import numpy as np


import math # core requirement for force calc




class obsavoid(Node):


    def __init__(self):
        super().__init__("lidar_subscriber")


        self.shutdown = False


        self.ForwardForce = 0.0
        self.LateralForce = 0.0


        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        )


        #subscribe to the cmd_vel topic to publish vel commands


        self.my_publisher = self.create_publisher(
            msg_type=TwistStamped,
            topic="/cmd_vel",
            qos_profile=10,
        )


        publish_rate = 10 # Hz


        self.timer = self.create_timer(
            timer_period_sec=1/publish_rate,
            callback=self.timer_callback
        )


        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")


    def lidar_callback(self, scan_data: LaserScan):


        tempcount = 0 #initialise counter, equivalent to angle


        RangeArray = np.array(scan_data.ranges[0:359])
        ForceArray = np.zeros(360, dtype=float)


        #Initialise tunable values, for later use in playing with repulsor settings.


        accel_parameter = -1
        obstacle_mass = 1
       
        #While loop to cycle through azimuth and assign forces from the objects detected


        while tempcount < 359:
            if RangeArray[tempcount] == "inf" or RangeArray[tempcount] == 0:
                ForceArray[tempcount] = 0


            elif RangeArray[tempcount] >= 1:
                ForceArray[tempcount] = 0


            elif 0.2 < RangeArray[tempcount] < 1:
                ForceArray[tempcount] = (accel_parameter*obstacle_mass)/RangeArray[tempcount]**2
               
            elif 0 <= RangeArray[tempcount]  <= 0.25:
                ForceArray[tempcount] = (accel_parameter*obstacle_mass)/RangeArray[tempcount]**3 ## power of 3 for additional weighting - proof of concept if it works


            self.LateralForce = self.LateralForce + math.sin(math.pi*tempcount/180)*ForceArray[tempcount]
            self.ForwardForce = self.ForwardForce + math.cos(math.pi*tempcount/180)*ForceArray[tempcount]


            tempcount += 1


    def on_shutdown(self):
        self.get_logger().info(
            "Stopping the robot..."
        )
        self.my_publisher.publish(TwistStamped())
        self.shutdown = True


    def timer_callback(self):
       
        topic_msg = TwistStamped()
        topic_msg.twist.linear.x = 0.0
        topic_msg.twist.linear.y = 0.0
        topic_msg.twist.linear.z = 0.0
        topic_msg.twist.angular.x = 0.0
        topic_msg.twist.angular.y = 0.0
        topic_msg.twist.angular.z = 0.0


        try:              
            Heading_Relative = math.atan(self.ForwardForce/self.LateralForce)-math.pi/2              
        except:
            Heading_Relative = 0.0


        ManoeuvreCutoff = 30 #define heading region where robot will attempt to move & turn simultaneously.
        AngleWeightMax = 1.86
        VelWeightMax = 0.26
             
        if abs(Heading_Relative) < ((math.pi/180)*ManoeuvreCutoff): # Just Playing around with cutoffs for manoeuvering
                     
            linear_velocity = VelWeightMax*math.cos(Heading_Relative)
            angular_velocity = AngleWeightMax*math.sin(Heading_Relative)
            print("Inside 30 deg")
           


        elif abs(Heading_Relative) >=  ((math.pi/180)*ManoeuvreCutoff):
           
            linear_velocity = VelWeightMax*math.cos(Heading_Relative)
            angular_velocity = AngleWeightMax*math.sin(Heading_Relative)


        #Publish movement message


        topic_msg.twist.linear.x = linear_velocity
        topic_msg.twist.angular.z = angular_velocity
        self.my_publisher.publish(topic_msg)


        self.get_logger().info(
            f"Linear Velocity: {topic_msg.twist.linear.x:.2f} [m/s], "
            f"Angular Velocity: {topic_msg.twist.angular.z:.2f} [rad/s].",
           
            throttle_duration_sec=1,
        )


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )


    lidar_subscriber = obsavoid()


    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        print("Shutdown request (Ctrl+C) detected...")
    finally:
        lidar_subscriber.on_shutdown() #Calls Shutdown Func
        while not lidar_subscriber:
            continue
        lidar_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

