#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.parameter import Parameter
from distance_warning_interfaces.action import CheckDistance
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rcl_interfaces.srv import GetParameters
import threading
import time


class DistanceActionServer(Node):
    def __init__(self):
        super().__init__('distance_action_server')
        
        self.threshold = 0.5
        self.DEFAULT_THRESHOLD = 0.5
        
        self.action_server = ActionServer(
            self,
            CheckDistance,
            'check_distance',
            self.execute_callback
        )
        
        self.param_event_subscription = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            10
        )
        
        self.param_client = self.create_client(GetParameters, '/set_threshold_service/get_parameters')
        self.initialize_threshold_from_service()
        
        self.get_logger().info(f'distance_action_server initialized with threshold: {self.threshold:.2f} m')

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
            
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')
        
        goal = goal_handle.request
        feedback_msg = CheckDistance.Feedback()
        result = CheckDistance.Result()
        
        current_threshold = self.threshold
        
        self.get_logger().info(f'Using threshold: {current_threshold:.2f} m for distance check')
        
        if goal.distance_to_check < 0.0:
            self.get_logger().warn(f'Invalid distance value: {goal.distance_to_check:.2f} m')
            goal_handle.abort()
            result.is_safe = False
            return result
        
        steps = 5
        for i in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                result.is_safe = False
                return result
            
            feedback_msg.feedback_msg = f'Processing step {i}/{steps} - Checking distance: {goal.distance_to_check:.2f} m'
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(feedback_msg.feedback_msg)
            
            time.sleep(0.5)
        
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal canceled')
            goal_handle.canceled()
            result.is_safe = False
            return result
        
        result.is_safe = goal.distance_to_check >= current_threshold
        goal_handle.succeed()
        
        if result.is_safe:
            self.get_logger().info(f'Distance check completed: SAFE ({goal.distance_to_check:.2f} m >= {current_threshold:.2f} m)')
        else:
            self.get_logger().warn(f'Distance check completed: UNSAFE ({goal.distance_to_check:.2f} m < {current_threshold:.2f} m)')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DistanceActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main() 