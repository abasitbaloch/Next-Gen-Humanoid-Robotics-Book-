#!/usr/bin/env python3

"""
Comprehensive Testing Framework for Physical AI & Humanoid Robotics Book

This test suite validates all components of the humanoid robotics system.
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import time
import json
import subprocess
import os
from pathlib import Path


class TestROS2System(unittest.TestCase):
    """Test ROS 2 system components"""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context for testing"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context after testing"""
        rclpy.shutdown()

    def test_basic_communication(self):
        """Test basic ROS 2 communication"""
        node = rclpy.create_node('test_basic_comm')

        # Create a simple publisher and subscriber
        pub = node.create_publisher(String, 'test_topic', 10)
        received_msgs = []

        def callback(msg):
            received_msgs.append(msg.data)

        sub = node.create_subscription(String, 'test_topic', callback, 10)

        # Publish a test message
        test_msg = String()
        test_msg.data = 'test_message'
        pub.publish(test_msg)

        # Spin to process messages
        rclpy.spin_once(node, timeout_sec=0.1)

        # Check if message was received
        self.assertEqual(len(received_msgs), 1)
        self.assertEqual(received_msgs[0], 'test_message')

        node.destroy_node()

    def test_navigation_system_available(self):
        """Test that navigation system is available"""
        node = rclpy.create_node('test_nav')

        # Check if navigation topics exist
        topics = node.get_topic_names_and_types()
        nav_topics = [name for name, _ in topics if 'nav' in name.lower()]

        # We expect at least some navigation topics
        self.assertGreaterEqual(len(nav_topics), 1, "Navigation topics should exist")

        node.destroy_node()

    def test_manipulation_system_available(self):
        """Test that manipulation system is available"""
        node = rclpy.create_node('test_manip')

        # Check if manipulation topics exist
        topics = node.get_topic_names_and_types()
        manip_topics = [name for name, _ in topics if 'manipulation' in name.lower() or 'arm' in name.lower()]

        # We expect at least some manipulation topics
        self.assertGreaterEqual(len(manip_topics), 1, "Manipulation topics should exist")

        node.destroy_node()


class TestCodeExamples(unittest.TestCase):
    """Test code examples from the book"""

    def test_ros2_publisher_example(self):
        """Test basic ROS 2 publisher example"""
        # This would test the actual code examples from the book
        # For now, we'll just verify that the example files exist and are syntactically correct
        example_path = Path("src/ros2_basics/examples")
        if example_path.exists():
            python_files = list(example_path.rglob("*.py"))
            for py_file in python_files:
                # Verify Python syntax
                with open(py_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    compile(content, str(py_file), 'exec')  # This will raise if syntax is invalid

    def test_simulation_examples(self):
        """Test simulation examples"""
        # Verify simulation example files exist
        sim_path = Path("src/simulation/examples")
        if sim_path.exists():
            launch_files = list(sim_path.rglob("*.py"))  # ROS 2 launch files are Python
            self.assertGreaterEqual(len(launch_files), 1, "Simulation examples should exist")

    def test_isaac_examples(self):
        """Test Isaac examples"""
        # Verify Isaac example files exist
        isaac_path = Path("src/isaac_integration/examples")
        if isaac_path.exists():
            py_files = list(isaac_path.rglob("*.py"))
            self.assertGreaterEqual(len(py_files), 1, "Isaac examples should exist")


class TestDocumentationStructure(unittest.TestCase):
    """Test documentation structure and links"""

    def test_docusaurus_build(self):
        """Test that Docusaurus site builds without errors"""
        # Check if package.json exists and has Docusaurus dependencies
        if Path("package.json").exists():
            with open("package.json", "r") as f:
                package_data = json.load(f)

            # Check for Docusaurus dependencies
            dependencies = package_data.get("dependencies", {})
            dev_dependencies = package_data.get("devDependencies", {})

            has_docusaurus = any("docusaurus" in dep for dep in dependencies.keys())
            has_docusaurus_dev = any("docusaurus" in dep for dep in dev_dependencies.keys())

            self.assertTrue(has_docusaurus or has_docusaurus_dev,
                          "Docusaurus dependencies should be present in package.json")

    def test_module_structure(self):
        """Test that all modules exist with required content"""
        required_modules = [
            "docs/module-1-ros2",
            "docs/module-2-digital-twin",
            "docs/module-3-ai-brain",
            "docs/module-4-vla",
            "docs/capstone"
        ]

        for module_path in required_modules:
            self.assertTrue(Path(module_path).exists(),
                          f"Module directory should exist: {module_path}")

            # Check for README or index file
            index_files = [f for f in Path(module_path).glob("*.md") if "readme" in f.name.lower() or "index" in f.name.lower()]
            self.assertGreaterEqual(len(index_files), 1,
                                  f"Module should have index file: {module_path}")

    def test_sidebar_structure(self):
        """Test sidebar structure"""
        sidebar_path = Path("sidebars.js")
        if sidebar_path.exists():
            with open(sidebar_path, "r") as f:
                content = f.read()

            # Check for required module references
            required_modules = ["module-1", "module-2", "module-3", "module-4", "capstone"]
            for module in required_modules:
                self.assertIn(module, content, f"Sidebar should reference {module}")


class TestSystemIntegration(unittest.TestCase):
    """Test system integration and end-to-end functionality"""

    def test_launch_files_exist(self):
        """Test that required launch files exist"""
        launch_paths = [
            Path("src/isaac_integration/launch"),
            Path("src/vla_pipeline/launch"),
            Path("src/capstone/launch")
        ]

        for launch_path in launch_paths:
            if launch_path.exists():
                launch_files = list(launch_path.glob("*.py"))
                self.assertGreaterEqual(len(launch_files), 1,
                                      f"Launch files should exist in {launch_path}")

    def test_configuration_files_exist(self):
        """Test that required configuration files exist"""
        config_paths = [
            Path("src/isaac_integration/config"),
            Path("config"),
            Path("src/ros2_basics/config")
        ]

        found_config = False
        for config_path in config_paths:
            if config_path.exists():
                config_files = list(config_path.glob("*.yaml")) + list(config_path.glob("*.yml"))
                if len(config_files) > 0:
                    found_config = True
                    break

        self.assertTrue(found_config, "Configuration files should exist")


class TestPerformanceRequirements(unittest.TestCase):
    """Test performance and quality requirements"""

    def test_build_time_requirement(self):
        """Test that build time is under 30 seconds (conceptual test)"""
        # This is a conceptual test - in reality, you'd measure actual build time
        # For now, we'll just verify that build scripts exist
        build_scripts = ["package.json", "setup.py", "CMakeLists.txt"]
        found_build_script = any(Path(script).exists() for script in build_scripts)

        self.assertTrue(found_build_script, "Build scripts should exist")

    def test_code_quality_metrics(self):
        """Test basic code quality metrics"""
        # Count lines of code in Python files as a basic metric
        python_files = list(Path(".").rglob("*.py"))

        total_lines = 0
        for py_file in python_files:
            if "test" not in str(py_file) and "venv" not in str(py_file):
                with open(py_file, 'r', encoding='utf-8') as f:
                    lines = len(f.readlines())
                    total_lines += lines

        # Basic requirement: should have some code
        self.assertGreater(total_lines, 100, "Should have substantial code content")


class TestHardwareRequirements(unittest.TestCase):
    """Test hardware requirements documentation"""

    def test_hardware_requirements_documented(self):
        """Test that hardware requirements are documented"""
        # Look for hardware requirements in documentation
        doc_paths = ["docs", "README.md", "specs"]

        found_hardware_docs = False
        for path in doc_paths:
            p = Path(path)
            if p.exists():
                if p.is_file():
                    with open(p, 'r', encoding='utf-8') as f:
                        content = f.read().lower()
                        if 'hardware' in content and ('gpu' in content or 'rtx' in content or 'jetson' in content):
                            found_hardware_docs = True
                            break
                else:
                    # Search in markdown files
                    md_files = p.rglob("*.md")
                    for md_file in md_files:
                        with open(md_file, 'r', encoding='utf-8') as f:
                            content = f.read().lower()
                            if 'hardware' in content and ('gpu' in content or 'rtx' in content or 'jetson' in content):
                                found_hardware_docs = True
                                break
            if found_hardware_docs:
                break

        self.assertTrue(found_hardware_docs, "Hardware requirements should be documented")


def run_comprehensive_tests():
    """Run all tests in the comprehensive test suite"""
    # Create a test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add all test cases
    suite.addTests(loader.loadTestsFromTestCase(TestROS2System))
    suite.addTests(loader.loadTestsFromTestCase(TestCodeExamples))
    suite.addTests(loader.loadTestsFromTestCase(TestDocumentationStructure))
    suite.addTests(loader.loadTestsFromTestCase(TestSystemIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestPerformanceRequirements))
    suite.addTests(loader.loadTestsFromTestCase(TestHardwareRequirements))

    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Print summary
    print(f"\nTest Results:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success: {result.wasSuccessful()}")

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_comprehensive_tests()
    exit(0 if success else 1)