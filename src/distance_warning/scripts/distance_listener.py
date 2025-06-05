#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from distance_warning_interfaces.msg import Distance
from rcl_interfaces.msg import ParameterEvent, ParameterType


class DistanceListener(Node):
    def __init__(self):
        super().__init__('distance_listener')
        
        self.threshold = 0.5
        self.DEFAULT_THRESHOLD = 0.5
        
        self.subscription = self.create_subscription(
            Distance,
            'distance_topic',
            self.distance_callback,
            10
        )
        
        self.param_event_subscription = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            10
        )
        
        self.param_client = self.create_client(GetParameters, '/set_threshold_service/get_parameters')
        self.initialize_threshold_from_service()
        
        self.get_logger().info(f'distance_listener initialized with threshold: {self.threshold:.2f} m')

    def initialize_threshold_from_service(self):
        timeout_sec = 5.0
        self.get_logger().info(f'Waiting for set_threshold_service to be available with timeout: {timeout_sec}s')
        
        if not self.param_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn('set_threshold_service not available within timeout')
            self.threshold = self.DEFAULT_THRESHOLD
            return
        
        future = self.param_client.call_async(GetParameters.Request(names=['threshold']))
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if not future.done() or not future.result():
            self.get_logger().warn('Timeout waiting for parameter response')
            self.threshold = self.DEFAULT_THRESHOLD
            return
        
        parameters = future.result().values
        if not parameters:
            self.get_logger().warn('No parameters found')
            self.threshold = self.DEFAULT_THRESHOLD
            return
        
        self.threshold = float(parameters[0].double_value)
        self.get_logger().info(f'Got initial threshold: {self.threshold:.2f} m from set_threshold_service')
        

    def parameter_event_callback(self, msg):
        if msg.node == '/set_threshold_service':
            if next(filter(lambda param: param.name == 'threshold', msg.changed_parameters), None):
                self.threshold = float(next(filter(lambda param: param.name == 'threshold', msg.changed_parameters)).value.double_value)
                self.get_logger().info(f'Threshold updated via parameter event: {self.threshold:.2f} m')
            elif next(filter(lambda param: param.name == 'threshold', msg.new_parameters), None):
                self.threshold = float(next(filter(lambda param: param.name == 'threshold', msg.new_parameters)).value.double_value)
                self.get_logger().info(f'Threshold set via parameter event: {self.threshold:.2f} m')

    def distance_callback(self, msg):
        self.get_logger().info(f'Received distance: {msg.distance:.2f} m (threshold: {self.threshold:.2f} m)')
        
        if msg.distance < self.threshold:
            self.get_logger().warn(f'Warning: Object too close! Distance: {msg.distance:.2f} m < Threshold: {self.threshold:.2f} m')
        else:
            self.get_logger().info(f'Distance safe: {msg.distance:.2f} m >= {self.threshold:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main() 