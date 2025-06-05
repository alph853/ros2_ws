#!/bin/bash

# Autonomous UAV System Build Script
# This script builds the complete autonomous UAV system

set -e  # Exit on any error

echo "=========================================="
echo "Building Autonomous UAV System"
echo "=========================================="

# Check if we're in a ROS2 workspace
if [ ! -f "src/autonomous_uav/package.xml" ] && [ ! -f "src/autonomous_uav_interfaces/package.xml" ]; then
    echo "Error: Please run this script from the ROS2 workspace root directory"
    echo "Expected structure: workspace/src/autonomous_uav/"
    exit 1
fi

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble sourced"
else
    echo "Error: ROS2 Humble not found. Please install ROS2 Humble first."
    exit 1
fi

# Install dependencies
echo ""
echo "Installing dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Clean previous build (optional)
if [ "$1" = "clean" ]; then
    echo ""
    echo "Cleaning previous build..."
    rm -rf build/ install/ log/
    echo "✓ Clean completed"
fi

# Build the packages
echo ""
echo "Building packages..."
echo "Building autonomous_uav_interfaces..."
colcon build --packages-select autonomous_uav_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "Building autonomous_uav..."
colcon build --packages-select autonomous_uav --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check build status
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build completed successfully!"
    echo "=========================================="
    echo ""
    echo "To use the system:"
    echo "1. Source the workspace: source install/setup.bash"
    echo "2. Launch the system: ros2 launch autonomous_uav autonomous_uav_system.launch.py"
    echo ""
    echo "For more information, see src/autonomous_uav/README.md"
else
    echo ""
    echo "=========================================="
    echo "✗ Build failed!"
    echo "=========================================="
    echo ""
    echo "Please check the error messages above and fix any issues."
    exit 1
fi 