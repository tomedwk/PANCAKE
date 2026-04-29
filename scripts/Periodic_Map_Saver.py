#!/usr/bin/env python3

# Saves the Lidar data every X seconds based on the variable Interval

import rclpy
from rclpy.node import Node
import subprocess
import os


class MapSaver(Node):

    def __init__(self):
        super().__init__("map_saver")

        # Folder to save maps
        self.save_dir = os.path.expanduser("~/ros2_ws/src/ele434_team15_2026/maps")
        os.makedirs(self.save_dir, exist_ok=True)

        # Save interval (seconds)
        self.interval = 25

        # Timer (same structure as your example)
        self.timer = self.create_timer(
            timer_period_sec=self.interval,
            callback=self.timer_callback
        )

        self.get_logger().info(
            f"'{self.get_name()}' node started. Saving maps every {self.interval} seconds."
        )

    def timer_callback(self):
        map_path = os.path.join(self.save_dir, f"explore_map")

        self.get_logger().info(f"Saving map: {map_path}")

        try:
            subprocess.run([
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", map_path
            ], check=True)

            self.get_logger().info("Map saved successfully.")

        except subprocess.CalledProcessError:
            self.get_logger().error("Failed to save map.")


def main(args=None):
    rclpy.init(args=args)

    node = MapSaver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
