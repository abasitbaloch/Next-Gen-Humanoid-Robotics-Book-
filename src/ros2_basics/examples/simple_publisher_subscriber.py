#!/usr/bin/env python3
"""
Simple Publisher/Subscriber Example for ROS 2

This example demonstrates the basic publisher-subscriber pattern in ROS 2.
It includes both a publisher node that sends messages and a subscriber node
that receives and processes those messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


class SimplePublisher(Node):
    """
    A simple publisher node that sends messages to a topic.
    """
    def __init__(self, message="Hello from publisher", publish_rate=1.0):
        super().__init__('simple_publisher')

        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Store the message to publish
        self.message = message

        # Create a timer to publish messages at regular intervals
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for messages
        self.i = 0

        self.get_logger().info(
            f'Publisher node initialized. Publishing to topic "chatter" '
            f'at {publish_rate} Hz with message: "{message}"'
        )

    def timer_callback(self):
        """Callback function that publishes messages at regular intervals."""
        msg = String()
        msg.data = f'{self.message}: {self.i}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1


class SimpleSubscriber(Node):
    """
    A simple subscriber node that receives messages from a topic.
    """
    def __init__(self):
        super().__init__('simple_subscriber')

        # Create a subscription to the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10  # QoS history depth
        )

        # Prevent unused variable warning
        self.subscription  # type: ignore

        self.get_logger().info('Subscriber node initialized. Listening to topic "chatter"...')

    def listener_callback(self, msg):
        """
        Callback function that processes incoming messages.

        Args:
            msg (String): The received message
        """
        self.get_logger().info(f'I heard: [{msg.data}]')


def main(args=None):
    """
    Main function that initializes ROS 2, creates nodes, and spins them.

    Args:
        args: Command line arguments (passed to rclpy.init())
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    try:
        # Create both publisher and subscriber nodes
        publisher_node = SimplePublisher(message="Hello World", publish_rate=2.0)
        subscriber_node = SimpleSubscriber()

        # Create an executor to manage both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher_node)
        executor.add_node(subscriber_node)

        # Log information about the nodes
        publisher_node.get_logger().info('Both publisher and subscriber nodes created. Starting execution...')

        try:
            # Spin both nodes until interrupted
            executor.spin()
        except KeyboardInterrupt:
            publisher_node.get_logger().info('Keyboard interrupt received, shutting down...')
        finally:
            # Clean up
            executor.shutdown()
            publisher_node.destroy_node()
            subscriber_node.destroy_node()

    finally:
        # Shutdown ROS 2
        rclpy.shutdown()


def publisher_only_main(args=None):
    """
    Main function that runs only the publisher node.

    Args:
        args: Command line arguments (passed to rclpy.init())
    """
    rclpy.init(args=args)

    try:
        publisher_node = SimplePublisher(message="Publisher Only Mode", publish_rate=1.0)

        try:
            rclpy.spin(publisher_node)
        except KeyboardInterrupt:
            publisher_node.get_logger().info('Keyboard interrupt received, shutting down...')
        finally:
            publisher_node.destroy_node()

    finally:
        rclpy.shutdown()


def subscriber_only_main(args=None):
    """
    Main function that runs only the subscriber node.

    Args:
        args: Command line arguments (passed to rclpy.init())
    """
    rclpy.init(args=args)

    try:
        subscriber_node = SimpleSubscriber()

        try:
            rclpy.spin(subscriber_node)
        except KeyboardInterrupt:
            subscriber_node.get_logger().info('Keyboard interrupt received, shutting down...')
        finally:
            subscriber_node.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    """
    Entry point for the example.

    Usage:
        - Run both publisher and subscriber: python3 simple_publisher_subscriber.py
        - Run publisher only: python3 simple_publisher_subscriber.py --publisher-only
        - Run subscriber only: python3 simple_publisher_subscriber.py --subscriber-only
    """
    if len(sys.argv) > 1:
        if sys.argv[1] == '--publisher-only':
            publisher_only_main()
        elif sys.argv[1] == '--subscriber-only':
            subscriber_only_main()
        else:
            print(f"Unknown argument: {sys.argv[1]}")
            print("Usage:")
            print("  Run both: python3 simple_publisher_subscriber.py")
            print("  Publisher only: python3 simple_publisher_subscriber.py --publisher-only")
            print("  Subscriber only: python3 simple_publisher_subscriber.py --subscriber-only")
    else:
        main()