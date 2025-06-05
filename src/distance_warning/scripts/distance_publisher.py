#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from distance_warning_interfaces.msg import Distance
import random


class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        
        self.publisher = self.create_publisher(Distance, 'distance_topic', 10)
        self.timer     = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(f'Distance Publisher initialized, publishing at 1 Hz')

    def timer_callback(self):
        msg = Distance()
        msg.distance = random.uniform(0.1, 1.5)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published distance: {msg.distance:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main() 