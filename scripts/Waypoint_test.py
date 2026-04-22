#!/usr/bin/env python3
# attempt at moving between waypoint 

import rclpy 
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

class MoveWaypoint(Node): 

    def __init__(self):
        super().__init__("move_waypoint") 

        self.my_publisher = self.create_publisher(
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
      publish_vel(0.1, 0)


    def publish_vel(self, linear_velocity, angular_velocity):

      topic_msg = TwistStamped() 
      topic_msg.twist.linear.x = linear_velocity
      topic_msg.twist.angular.z = angular_velocity
      self.my_publisher.publish(topic_msg) 

      self.get_logger().info( 
          f"Linear Velocity: {topic_msg.twist.linear.x:.2f} [m/s], "
          f"Angular Velocity: {topic_msg.twist.angular.z:.2f} [rad/s].",
          throttle_duration_sec=1, 
      )

def main(args=None): 
    rclpy.init(args=args)
    my_simple_publisher = SimplePublisher()
    rclpy.spin(my_simple_publisher)
    my_simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
