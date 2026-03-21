# PaceCat 2D LiDAR Suite

The PaceCat 2D LiDAR development kit supports both Windows and Linux platforms, providing a C++ SDK along with ROS and ROS 2 drivers.

&gt; 🌐 [中文文档](./README_CN.md)

## Project Structure

| Directory | Description | Documentation |
|:---|:---|:---|
| [`sdk`](./sdk/) | C++ SDK core library (Windows/Linux) | [SDK Docs](./sdk/README.md) |
| [`ros`](./ros/) | ROS driver package (Noetic/Melodic) | [ROS Docs](./ros/src/README.md) |
| [`ros2`](./ros2/) | ROS 2 driver package (Foxy/Humble) | [ROS 2 Docs](./ros2/README.md) |

&gt; 💡 Choose the appropriate subproject based on your development platform and requirements.

## Quick Start

### System Requirements

- **SDK**:
  - Linux: GCC/G++ (recommended 7.5 or newer)
  - Windows: Visual Studio 2013 or later

- **ROS**:
  - Ubuntu 18.04 + ROS Melodic
  - Ubuntu 20.04 + ROS Noetic

- **ROS 2**:
  - Ubuntu 20.04 + ROS 2 Foxy
  - Ubuntu 22.04 + ROS 2 Humble

&gt; For detailed build and usage instructions, please refer to the documentation in each subproject.