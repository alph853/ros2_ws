#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from distance_warning_interfaces.srv import SetThreshold
from rclpy.parameter import Parameter


class SetThresholdService(Node):
    def __init__(self):
        super().__init__('set_threshold_service')
        
        self.MIN_THRESHOLD = 0.1
        self.MAX_THRESHOLD = 1.5
        
        self.declare_parameter('threshold', 0.5)
        self.current_threshold = self.get_parameter('threshold').get_parameter_value().double_value
        
        self.service = self.create_service(
            SetThreshold,
            'set_threshold',
            self.handle_set_threshold
        )
        
        self.get_logger().info(f'Set Threshold Service initialized with threshold: {self.current_threshold:.2f} m')

    def handle_set_threshold(self, request, response):
        new_threshold = float(request.threshold)
        
        if new_threshold < self.MIN_THRESHOLD:
            new_threshold = self.MIN_THRESHOLD
            self.get_logger().warn(f'Requested threshold too low, setting to minimum: {self.MIN_THRESHOLD:.2f} m')
        elif new_threshold > self.MAX_THRESHOLD:
            new_threshold = self.MAX_THRESHOLD
            self.get_logger().warn(f'Requested threshold too high, setting to maximum: {self.MAX_THRESHOLD:.2f} m')
        
        param = Parameter('threshold', Parameter.Type.DOUBLE, new_threshold)
        self.set_parameters([param])
        self.current_threshold = new_threshold
        
        response.success = True
        
        self.get_logger().info(f'Threshold updated to: {self.current_threshold:.2f} m. Other nodes will get this value when they request it.')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SetThresholdService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 