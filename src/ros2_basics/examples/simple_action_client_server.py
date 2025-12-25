#!/usr/bin/env python3
"""
Simple Action Client/Server Example for ROS 2

This example demonstrates the basic action-client pattern in ROS 2.
It includes both an action server that executes long-running tasks and an action client
that sends goals and receives feedback and results.
"""

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Using the Fibonacci action as an example (built into example_interfaces)
from example_interfaces.action import Fibonacci


class SimpleActionServer(Node):
    """
    A simple action server that calculates Fibonacci sequences.
    """
    def __init__(self):
        super().__init__('simple_action_server')

        # Create an action server
        # Using ReentrantCallbackGroup to allow multiple callbacks to run simultaneously
        callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=callback_group
        )

        self.get_logger().info('Action server initialized. Ready to receive goals...')

    def destroy_node(self):
        """Override destroy_node to properly shutdown the action server."""
        self._action_server.destroy()
        super().destroy_node()

    async def execute_callback(self, goal_handle):
        """
        Execute callback that processes the goal and sends feedback.

        Args:
            goal_handle: The goal handle containing the request

        Returns:
            Fibonacci.Result: The result of the action
        """
        self.get_logger().info(f'Executing goal: order = {goal_handle.request.order}')

        # Initialize the Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Send initial feedback
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'Initial feedback: {feedback_msg.sequence}')

        # Calculate Fibonacci sequence up to the requested order
        for i in range(1, goal_handle.request.order):
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            # Calculate next Fibonacci number
            next_fib = feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            feedback_msg.sequence.append(next_fib)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Simulate some processing time
            from time import sleep
            sleep(0.5)

        # Check if the goal was canceled during execution
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled during execution')
            result = Fibonacci.Result()
            result.sequence = feedback_msg.sequence
            return result

        # Complete the goal successfully
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'Goal succeeded with result: {result.sequence}')
        return result


class SimpleActionClient(Node):
    """
    A simple action client that sends goals to the action server.
    """
    def __init__(self):
        super().__init__('simple_action_client')

        # Create an action client
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

        self.get_logger().info('Action client initialized.')

    def send_goal(self, order):
        """
        Send a goal to the action server.

        Args:
            order (int): The order of the Fibonacci sequence to calculate

        Returns:
            Future: A future that resolves when the goal is completed
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create the goal request
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: order = {order}')

        # Send the goal asynchronously with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add a callback for when the goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        """
        Callback for when the goal response is received.

        Args:
            future: The future containing the goal handle
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')

        # Get the result when the goal completes
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback for receiving feedback from the action server.

        Args:
            feedback_msg: The feedback message from the server
        """
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

    def get_result_callback(self, future):
        """
        Callback for when the result is received.

        Args:
            future: The future containing the result
        """
        result = future.result().result
        self.get_logger().info(f'Result received: {result.sequence}')


def run_action_server():
    """
    Function to run the action server.
    """
    rclpy.init()

    try:
        action_server = SimpleActionServer()

        try:
            rclpy.spin(action_server)
        except KeyboardInterrupt:
            action_server.get_logger().info('Action server interrupted, shutting down...')
        finally:
            action_server.destroy_node()

    finally:
        rclpy.shutdown()


def run_action_client():
    """
    Function to run the action client and send some goals.
    """
    rclpy.init()

    try:
        action_client = SimpleActionClient()

        # Send several goals
        orders = [5, 3, 7]

        for order in orders:
            # Send the goal
            future = action_client.send_goal(order)

            # Wait for the result (in a real application, you might want to do other things)
            rclpy.spin_until_future_complete(action_client, future)

            # Wait a bit between goals
            from time import sleep
            sleep(2.0)

        action_client.destroy_node()

    finally:
        rclpy.shutdown()


def main(args=None):
    """
    Main function that demonstrates both action server and client in one process.

    Note: This is mainly for demonstration. In practice, action servers and clients
    typically run in separate processes/nodes.
    """
    rclpy.init(args=args)

    try:
        # Create both action server and client nodes
        action_server = SimpleActionServer()
        action_client = SimpleActionClient()

        # Create an executor to manage both nodes
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)
        executor.add_node(action_client)

        # Send a goal from the client
        def send_goal_from_main():
            """Function to send a goal from the main thread."""
            import time
            time.sleep(2.0)  # Wait a bit for everything to initialize

            # Send a goal
            action_client.send_goal(6)

        # Start sending a goal in a separate thread
        from threading import Thread
        goal_thread = Thread(target=send_goal_from_main)
        goal_thread.start()

        try:
            # Spin both nodes
            executor.spin()
        except KeyboardInterrupt:
            action_server.get_logger().info('Interrupted, shutting down...')
        finally:
            goal_thread.join(timeout=1.0)  # Wait for the goal thread to finish
            executor.shutdown()
            action_server.destroy_node()
            action_client.destroy_node()

    finally:
        rclpy.shutdown()


def server_main(args=None):
    """
    Main function that runs only the action server.

    Args:
        args: Command line arguments (passed to rclpy.init())
    """
    rclpy.init(args=args)

    try:
        action_server = SimpleActionServer()

        try:
            rclpy.spin(action_server)
        except KeyboardInterrupt:
            action_server.get_logger().info('Action server interrupted, shutting down...')
        finally:
            action_server.destroy_node()

    finally:
        rclpy.shutdown()


def client_main(args=None):
    """
    Main function that runs only the action client.

    Args:
        args: Command line arguments (passed to rclpy.init())
    """
    rclpy.init(args=args)

    try:
        action_client = SimpleActionClient()

        # Send a few goals
        orders = [4, 5, 6]

        for order in orders:
            future = action_client.send_goal(order)
            # Wait a bit between goals
            from time import sleep
            sleep(3.0)

        action_client.destroy_node()

    finally:
        rclpy.shutdown()


def advanced_client_example():
    """
    Example of a more advanced client that can cancel goals.
    """
    rclpy.init()

    try:
        action_client = SimpleActionClient()

        # Send a long goal that we might want to cancel
        future = action_client.send_goal(10)

        # Wait a bit, then cancel the goal
        from time import sleep
        sleep(2.0)

        # In a real scenario, you would have a way to get the goal handle
        # For this example, we'll just send another goal to demonstrate
        action_client.send_goal(3)

        # Wait for completion
        rclpy.spin_once(action_client, timeout_sec=5)

        action_client.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    """
    Entry point for the example.

    Usage:
        - Run server only: python3 simple_action_client_server.py --server
        - Run client only: python3 simple_action_client_server.py --client
        - Run advanced client: python3 simple_action_client_server.py --advanced-client
        - Run both (demo): python3 simple_action_client_server.py
    """
    if len(sys.argv) > 1:
        if sys.argv[1] == '--server':
            server_main()
        elif sys.argv[1] == '--client':
            client_main()
        elif sys.argv[1] == '--advanced-client':
            advanced_client_example()
        else:
            print(f"Unknown argument: {sys.argv[1]}")
            print("Usage:")
            print("  Run server: python3 simple_action_client_server.py --server")
            print("  Run client: python3 simple_action_client_server.py --client")
            print("  Advanced client: python3 simple_action_client_server.py --advanced-client")
            print("  Demo mode: python3 simple_action_client_server.py")
    else:
        # Demo mode - run both in one process (for demonstration only)
        main()