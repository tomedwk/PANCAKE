#!/usr/bin/env python3
# Code to set lidar detected objects as repulsors


import rclpy


from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from example_interfaces.msg import String
from ele434_team15_2026.msg import KeyInfo #import key info as a new type
from ele434_team15_2026.msg import LocusArray


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

        #Create Subsciber to Waypoint_Locus Topic

        self.Waypoint_Location = self.create_subscription(
            msg_type = LocusArray,
            topic="Waypoint_Locus",
            callback=self.Waypoint_callback,
            qos_profile=10,
        )

        self.Waypoint_Location= None

        #Create Subscriber to Key Info Topic to know when to initialise

        self.key_info=KeyInfo()

        self.key_info_subscriber = self.create_subscription(
            msg_type=KeyInfo, 
            topic="/key_info", 
            callback=self.key_info_callback, 
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

    def key_info_callback(self, key_info_message: KeyInfo): 
         # recieve key info data
         self.key_info=key_info_message

         self.get_logger().info(
             f"state: {self.key_info.state}",
              throttle_duration_sec=1, 
         )

    def Waypoint_callback(self, locus_info_message: LocusArray):
        self.Waypoint_Location = locus_info_message
 

    def lidar_callback(self, scan_data: LaserScan):

        if self.Waypoint_Location is None:
            self.get_logger().warn( 
            f"IDLE: Not in lidar_callback state",
            throttle_duration_sec=1, 
            ) 
        else:
            tempcount = 0 #initialise counter, equivalent to angle


            RangeArray = np.array(scan_data.ranges[0:359])
            ForceArray = np.zeros(360, dtype=float)


            centreangle = int(math.pi*self.Waypoint_Location.waypointangle/180)
            
            self.FrontRanges = np.array(scan_data.ranges[-18:19])
            #Initialise tunable values, for later use in playing with repulsor settings.


            accel_parameter = -1 # for obstacles to be repulsive
            way_accel_parameter = 1 # for waypoint attractive force
            obstacle_mass = 1
            waypoint_mass = 10
        
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
            
            #Generate attractive force for waypoint.
            Waypoint_Force = way_accel_parameter*waypoint_mass/(self.Waypoint_Location.waypointdistance**2)
            self.LateralForce = self.LateralForce + math.sin(math.pi*self.Waypoint_Location.waypointangle/180)*Waypoint_Force
            self.ForwardForce = self.ForwardForce + math.cos(math.pi*self.Waypoint_Location.waypointangle/180)*Waypoint_Force

        

    def on_shutdown(self):
        self.get_logger().info(
            "Stopping the robot..."
        )
        self.my_publisher.publish(TwistStamped())
        self.shutdown = True


    def timer_callback(self):
        if self.key_info.state == "Obstacle":

            topic_msg = TwistStamped()
            topic_msg.twist.linear.x = 0.0
            topic_msg.twist.linear.y = 0.0
            topic_msg.twist.linear.z = 0.0
            topic_msg.twist.angular.x = 0.0
            topic_msg.twist.angular.y = 0.0
            topic_msg.twist.angular.z = 0.0

            try:
                Heading_Relative = math.atan(abs(self.LateralForce)/abs(self.ForwardForce))              
            except:
                Heading_Relative = 0.0
            if self.LateralForce >= 0 and self.ForwardForce < 0:
                Heading_Relative = math.pi-Heading_Relative
            elif self.LateralForce < 0 and self.ForwardForce < 0:
                Heading_Relative = -1*(math.pi-Heading_Relative)
            elif self.LateralForce < 0 and self.ForwardForce >= 0:
                Heading_Relative = -Heading_Relative
        
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

            if self.FrontRanges(0) > (self.Waypoint_Location.waypointdistance+0.1) and self.FrontRanges(38) > (self.Waypoint_Location.waypointdistance+0.1)


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
