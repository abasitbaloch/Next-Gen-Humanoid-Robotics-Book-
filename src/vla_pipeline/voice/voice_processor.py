#!/usr/bin/env python3

"""
Voice Processing Node for Vision-Language-Action Pipeline

This node handles voice recognition and command processing for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import speech_recognition as sr
import threading
import queue
import json
import time


class VoiceProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Declare parameters
        self.declare_parameter('voice_model', 'base')
        self.declare_parameter('language', 'en-US')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)

        # Get parameters
        self.voice_model = self.get_parameter('voice_model').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value

        # Publishers
        self.voice_command_pub = self.create_publisher(String, '/voice_command', 10)
        self.llm_request_pub = self.create_publisher(String, '/llm_request', 10)

        # Subscribers
        self.voice_input_sub = self.create_subscription(
            String,
            '/voice_input',
            self.voice_input_callback,
            10
        )

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set energy threshold for ambient noise
        self.recognizer.energy_threshold = 4000

        # Processing queue
        self.processing_queue = queue.Queue()

        # Timer for processing voice commands
        self.voice_timer = self.create_timer(0.1, self.process_voice_commands)

        self.get_logger().info('Voice Processor initialized')

    def voice_input_callback(self, msg):
        """Callback for voice input (simulated or from audio)"""
        # In simulation, we might receive text commands directly
        # In real implementation, this would come from audio processing
        self.processing_queue.put(msg.data)

    def process_voice_commands(self):
        """Process voice commands from the queue"""
        try:
            while not self.processing_queue.empty():
                voice_data = self.processing_queue.get_nowait()
                self.process_voice_command(voice_data)
        except queue.Empty:
            pass

    def process_voice_command(self, voice_data):
        """Process a voice command and extract intent"""
        self.get_logger().info(f'Processing voice command: {voice_data}')

        # In a real implementation, this would use Whisper or similar
        # For now, we'll simulate command parsing
        command = self.parse_voice_command(voice_data)

        if command:
            # Publish the parsed command
            cmd_msg = String()
            cmd_msg.data = json.dumps(command)
            self.voice_command_pub.publish(cmd_msg)

            # Also send to LLM for planning
            llm_msg = String()
            llm_msg.data = json.dumps({
                'type': 'voice_command',
                'command': command,
                'raw_input': voice_data
            })
            self.llm_request_pub.publish(llm_msg)

            self.get_logger().info(f'Voice command processed: {command}')
        else:
            self.get_logger().warn(f'Could not parse voice command: {voice_data}')

    def parse_voice_command(self, voice_data):
        """Parse voice command and extract structured intent"""
        # Convert to lowercase for processing
        text = voice_data.lower().strip()

        # Define command patterns
        command_patterns = [
            {'pattern': 'move to the (.+)', 'action': 'navigate', 'object': 1},
            {'pattern': 'go to the (.+)', 'action': 'navigate', 'object': 1},
            {'pattern': 'move (.+)', 'action': 'navigate', 'object': 1},
            {'pattern': 'pick up the (.+)', 'action': 'grasp', 'object': 1},
            {'pattern': 'grasp the (.+)', 'action': 'grasp', 'object': 1},
            {'pattern': 'pick (.+)', 'action': 'grasp', 'object': 1},
            {'pattern': 'place the (.+)', 'action': 'place', 'object': 1},
            {'pattern': 'put down the (.+)', 'action': 'place', 'object': 1},
            {'pattern': 'stop', 'action': 'stop', 'object': None},
            {'pattern': 'halt', 'action': 'stop', 'object': None},
            {'pattern': 'look at (.+)', 'action': 'look_at', 'object': 1},
            {'pattern': 'find (.+)', 'action': 'find_object', 'object': 1},
        ]

        # Try to match patterns
        for pattern in command_patterns:
            import re
            match = re.search(pattern['pattern'], text)
            if match:
                command = {
                    'action': pattern['action'],
                    'raw_command': voice_data,
                    'timestamp': time.time()
                }

                if pattern['object'] is not None:
                    command['target'] = match.group(pattern['object'])

                return command

        # If no pattern matches, return a generic command
        return {
            'action': 'unknown',
            'raw_command': voice_data,
            'timestamp': time.time()
        }

    def start_listening(self):
        """Start listening for voice commands (in a real implementation)"""
        # This would be used in a real implementation with actual microphone
        def listen_loop():
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source)

            while True:
                try:
                    with self.microphone as source:
                        audio = self.recognizer.listen(source, timeout=1)

                    # Use Google Speech Recognition (or Whisper in real implementation)
                    text = self.recognizer.recognize_google(audio, language=self.language)
                    self.processing_queue.put(text)

                except sr.WaitTimeoutError:
                    # No speech detected, continue listening
                    continue
                except sr.UnknownValueError:
                    self.get_logger().warn('Could not understand audio')
                except sr.RequestError as e:
                    self.get_logger().error(f'Speech recognition error: {e}')
                    time.sleep(1)  # Wait before retrying

        # Start listening in a separate thread
        listener_thread = threading.Thread(target=listen_loop, daemon=True)
        listener_thread.start()


def main(args=None):
    rclpy.init(args=args)

    voice_processor = VoiceProcessor()

    try:
        # In simulation, we won't actually start listening
        # In real implementation, call voice_processor.start_listening()
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()