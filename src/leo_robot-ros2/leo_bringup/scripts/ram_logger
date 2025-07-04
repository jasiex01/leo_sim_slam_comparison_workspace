#!/usr/bin/env python3

import psutil
import csv
import time
import os
import rclpy
from rclpy.node import Node

TARGET_NODE_NAME = '/opt/ros/jazzy/lib/rtabmap_slam/rtabmap'  # Replace this

class MemoryLogger(Node):
    def __init__(self):
        super().__init__('memory_logger')
        self.log_interval = 5.0  # seconds
        self.output_file = os.path.expanduser('~/ros_node_memory_log.csv')

        with open(self.output_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'memory_MB'])

        self.timer = self.create_timer(self.log_interval, self.log_memory_usage)
        self.get_logger().info(f"Logging memory usage of '{TARGET_NODE_NAME}' to {self.output_file}")

    def log_memory_usage(self):
        pid = self.find_process_pid_by_name(TARGET_NODE_NAME)
        if pid is None:
            self.get_logger().warn(f"Process '{TARGET_NODE_NAME}' not found.")
            return

        try:
            process = psutil.Process(pid)
            mem_info = process.memory_info()
            memory_mb = mem_info.rss / (1024 * 1024)
            timestamp = time.time()

            with open(self.output_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, f"{memory_mb:.2f}"])
            
            self.get_logger().info(f"{TARGET_NODE_NAME}: {memory_mb:.2f} MB")
        except psutil.NoSuchProcess:
            self.get_logger().warn("Process exited before memory could be logged.")

    def find_process_pid_by_name(self, name):
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if name in proc.info['cmdline']:
                    return proc.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None


def main(args=None):
    rclpy.init(args=args)
    node = MemoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Memory logger stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
