#!/usr/bin/env python3

#Range Segmentation
#Author: Emily
#Purpose: Tune Collision Zone, generally be able to have different sections that it can use

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions  #handle shutdown requests

from sensor_msgs.msg import LaserScan  #Laser scan data type
import numpy as np #Used for data put togethers

class LidarSubscriber(Node): 

    def __init__(self): 
        super().__init__("lidar_subscriber")

        self.lidar_sub = self.create_subscription( #Subscription to LIDAR
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        ) 

        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

    def lidar_callback(self, scan_data: LaserScan): 
        #sense=[0,0,0,0]
        #Tuned to be collision zone
        collision_left = scan_data.ranges[0:19] 
        collision_right = scan_data.ranges[-18:] 
        collision_zone = np.array(collision_right + collision_left) #Convert to array rather then list

        front_left=scan_data.ranges[0:14]
        front_right=scan_data.ranges[-14:]
        front=np.array(front_right + front_left)


        left=np.array(scan_data.ranges[15:30])
        right=np.array(scan_data.ranges[-15:])

        valid_data = collision_zone[collision_zone != float("inf")] #filter out infinite ones
        valid_left=left[left !=float("inf")]
        valid_right=right[right !=float("inf")]
        valid_front=front[front != float("inf")]

        if np.shape(valid_data)[0] > 0: 
            collision_avg=valid_data.mean()
        else:
            collision_avg=float("nan")
        if np.shape(valid_front)[0] > 0: 
            sense[1]=1
        if np.shape(valid_left)[0] > 0: 
            sense[2]=1
        if np.shape(valid_right)[0] > 0: 
            sense[3]=1



         
        """ if np.shape(valid_data)[0] > 0: #Not an empty space
            single_point_average = valid_data.mean() #Average of all readings to prevent noise
        else: #if its an empty space
            single_point_average = float("nan") """ 

        self.get_logger().info(
            f"LiDAR Reading: (collision_zone): {sense[0]} front {sense[1]}, left {sense[2]}, right {sense[3]}",
            throttle_duration_sec = 1,
        ) #printing but only every 1 second

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown request (Ctrl+C) detected...")
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()