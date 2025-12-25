#!/usr/bin/env python3
"""
Simple Service/Client Example for ROS 2

This example demonstrates the basic service-client pattern in ROS 2.
It includes both a service server that responds to requests and a client
that sends requests and receives responses.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys
import time
from threading import Thread


class SimpleService(Node):
    """
    A simple service server that adds two integers.
    """
    def __init__(self):
        super().__init__('simple_service_server')

        # Create a service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Service server initialized. Ready to receive requests...')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that handles incoming service requests.

        Args:
            request (AddTwoInts.Request): The incoming request with two integers
            response (AddTwoInts.Response): The response to be sent back

        Returns:
            AddTwoInts.Response: The response with the sum
        """
        # Perform the calculation
        response.sum = request.a + request.b

        # Log the operation
        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b} = {response.sum}'
        )

        return response


class SimpleClient(Node):
    """
    A simple client that sends requests to the service.
    """
    def __init__(self):
        super().__init__('simple_service_client')

        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client initialized. Ready to send requests...')

    def send_request(self, a, b):
        """
        Send a request to the service and wait for the response.

        Args:
            a (int): First integer
            b (int): Second integer

        Returns:
            AddTwoInts.Response: The response from the service
        """
        # Create the request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Send the request asynchronously
        self.get_logger().info(f'Sending request: {a} + {b}')
        future = self.cli.call_async(request)

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response.sum}')
            return response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None


def run_service_server():
    """
    Function to run the service server in a separate thread.
    """
    rclpy.init()

    try:
        service_node = SimpleService()

        try:
            rclpy.spin(service_node)
        except KeyboardInterrupt:
            service_node.get_logger().info('Service server interrupted, shutting down...')
        finally:
            service_node.destroy_node()

    finally:
        rclpy.shutdown()


def run_client():
    """
    Function to run the client and send some requests.
    """
    rclpy.init()

    try:
        client_node = SimpleClient()

        # Send several requests
        test_cases = [
            (1, 2),
            (10, 20),
            (-5, 7),
            (100, -50),
            (0, 0)
        ]

        for a, b in test_cases:
            response = client_node.send_request(a, b)
            if response:
                expected = a + b
                if response.sum == expected:
                    client_node.get_logger().info(f'✓ Correct result: {response.sum}')
                else:
                    client_node.get_logger().error(
                        f'✗ Incorrect result: got {response.sum}, expected {expected}'
                    )
            else:
                client_node.get_logger().error(f'✗ Failed to get response for {a} + {b}')

            # Small delay between requests
            time.sleep(0.5)

        client_node.destroy_node()

    finally:
        rclpy.shutdown()


def main(args=None):
    """
    Main function that demonstrates both service and client in one process.

    Note: This is mainly for demonstration. In practice, services and clients
    typically run in separate processes/nodes.
    """
    rclpy.init(args=args)

    try:
        # Create both service and client nodes
        service_node = SimpleService()
        client_node = SimpleClient()

        # Create an executor to manage both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(service_node)
        executor.add_node(client_node)

        # In a real scenario, you would run the service in one process and
        # the client in another. For this example, we'll send a few requests
        # and then let the service continue running.

        def send_requests():
            """Function to send requests from the client node."""
            test_cases = [
                (5, 3),
                (15, 25),
                (-10, 10)
            ]

            for a, b in test_cases:
                # Create the request
                request = AddTwoInts.Request()
                request.a = a
                request.b = b

                # Send the request asynchronously
                client_node.get_logger().info(f'Sending request: {a} + {b}')
                future = client_node.cli.call_async(request)

                # Wait for the response
                rclpy.spin_until_future_complete(client_node, future)

                try:
                    response = future.result()
                    client_node.get_logger().info(f'Received response: {response.sum}')
                except Exception as e:
                    client_node.get_logger().error(f'Service call failed: {e}')

                # Small delay between requests
                time.sleep(1.0)

        # Send some initial requests
        send_requests()

        # Continue spinning to keep the service running
        try:
            executor.spin()
        except KeyboardInterrupt:
            service_node.get_logger().info('Interrupted, shutting down...')
        finally:
            executor.shutdown()
            service_node.destroy_node()
            client_node.destroy_node()

    finally:
        rclpy.shutdown()


def service_main(args=None):
    """
    Main function that runs only the service server.

    Args:
        args: Command line arguments (passed to rclpy.init())
    """
    rclpy.init(args=args)

    try:
        service_node = SimpleService()

        try:
            rclpy.spin(service_node)
        except KeyboardInterrupt:
            service_node.get_logger().info('Service server interrupted, shutting down...')
        finally:
            service_node.destroy_node()

    finally:
        rclpy.shutdown()


def client_main(args=None):
    """
    Main function that runs only the client.

    Args:
        args: Command line arguments (passed to rclpy.init())
    """
    rclpy.init(args=args)

    try:
        client_node = SimpleClient()

        # Send a few requests
        test_cases = [(7, 3), (20, 30), (-5, 15)]

        for a, b in test_cases:
            response = client_node.send_request(a, b)
            if response:
                client_node.get_logger().info(f'Result of {a} + {b} = {response.sum}')
            time.sleep(1.0)

        client_node.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    """
    Entry point for the example.

    Usage:
        - Run service only: python3 simple_service_client_server.py --service
        - Run client only: python3 simple_service_client_server.py --client
        - Run both (demo): python3 simple_service_client_server.py
    """
    if len(sys.argv) > 1:
        if sys.argv[1] == '--service':
            service_main()
        elif sys.argv[1] == '--client':
            client_main()
        else:
            print(f"Unknown argument: {sys.argv[1]}")
            print("Usage:")
            print("  Run service: python3 simple_service_client_server.py --service")
            print("  Run client: python3 simple_service_client_server.py --client")
            print("  Demo mode: python3 simple_service_client_server.py")
    else:
        # Demo mode - run both in one process (for demonstration only)
        main()