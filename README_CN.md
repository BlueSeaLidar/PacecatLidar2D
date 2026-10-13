# 蓝海2D激光雷达套件

蓝海2D激光雷达开发套件，支持 Windows/Linux 平台，提供 C++ SDK 及 ROS/ROS2 驱动。

&gt; 🌐 [English Document](./README.md)

## 项目结构

| 目录 | 说明 | 文档 |
|:---|:---|:---|
| [`sdk`](./sdk/) | C++ SDK 核心库（Windows/Linux） | [SDK文档](./sdk/README_CN.md) |
| [`ros`](./ros/) | ROS 驱动包（Noetic/Melodic） | [ROS文档](./ros/src/README_CN.md) |
| [`ros2`](./ros2/) | ROS2 驱动包（Foxy/Humble） | [ROS2文档](./ros2/README_CN.md) |
| [`ota`](./ota/) | 固件升级工具（串口/UDP） | [串口](./ota/uart/README_CN.md) · [UDP](./ota/udp/README_CN.md) |

&gt; 💡 根据你的开发平台和需求，选择对应的子项目开始使用。

## 快速开始

### 环境要求

- **SDK**:  
  - Linux: GCC/G++ (建议 7.5+)  
  - Windows: Visual Studio 2013 或更高版本

- **ROS**:  
  - Ubuntu 18.04 + ROS Melodic  
  - Ubuntu 20.04 + ROS Noetic

- **ROS 2**:  
  - Ubuntu 20.04 + ROS 2 Foxy  
  - Ubuntu 22.04 + ROS 2 Humble

&gt; 更详细的构建与运行步骤，请参见各子项目的文档。
