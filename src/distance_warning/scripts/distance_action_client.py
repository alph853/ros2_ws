#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from distance_warning_interfaces.action import CheckDistance
import random


class DistanceActionClient(Node):
    def __init__(self):
        super().__init__('distance_action_client')
        
        self.action_client = ActionClient(self, CheckDistance, 'check_distance')
        self.timer         = self.create_timer(5.0, self.timer_callback)
        
        self.get_logger().info(f'Distance Action Client initialized, sending goals every 5 seconds')

    def timer_callback(self):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available, skipping goal')
            return
        
        random_distance = random.uniform(0.1, 1.5)
        self.send_goal(random_distance)

    def send_goal(self, distance):
        goal_msg = CheckDistance.Goal()
        goal_msg.distance_to_check = distance
        
        self.get_logger().info(f'Sending goal to check distance: {distance:.2f} m')
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by server')
            return
        
        self.get_logger().info('Goal accepted by server, waiting for result')
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.feedback_msg}')

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            if result.is_safe:
                self.get_logger().info('Result: Distance is SAFE!')
            else:
                self.get_logger().warn('Result: Distance is UNSAFE - WARNING!')
        elif status == 2:  # CANCELED
            self.get_logger().error('Goal was canceled')
        elif status == 3:  # ABORTED
            self.get_logger().error('Goal was aborted')
        else:
            self.get_logger().error(f'Unknown result status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main() 