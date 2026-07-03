# ROS2 机器人数据采集与测试知识手册

> 本文档系统梳理 ROS2 环境下机器人数据采集、录制、回放、测试所需的全部核心知识、命令与操作流程。

---

## 目录

1. [ROS2 基础环境](#1-ros2-基础环境)
2. [工作空间与包管理](#2-工作空间与包管理)
3. [节点与通信核心概念](#3-节点与通信核心概念)
4. [Topic 数据采集](#4-topic-数据采集)
5. [rosbag2 数据录制与回放](#5-rosbag2-数据录制与回放)
6. [传感器数据采集](#6-传感器数据采集)
7. [参数与配置管理](#7-参数与配置管理)
8. [Launch 系统](#8-launch-系统)
9. [TF 与坐标系](#9-tf-与坐标系)
10. [日志与调试](#10-日志与调试)
11. [自动化测试框架](#11-自动化测试框架)
12. [性能分析与监控](#12-性能分析与监控)
13. [数据后处理与可视化](#13-数据后处理与可视化)
14. 数据清洗
16. [DDS 与通信配置](#14-dds-与通信配置)
17. [常见数据采集场景速查](#15-常见数据采集场景速查)

---

## 1. ROS2 基础环境

### 1.1 环境变量

```bash
# 加载 ROS2 环境（每次打开终端需执行）
source /opt/ros/<distro>/setup.bash    # distro: humble, iron, jazzy, rolling

# 加载自定义工作空间
source ~/ros2_ws/install/setup.bash

# 查看当前 ROS 版本
echo $ROS_DISTRO
echo $ROS_VERSION         # 应为 2

# 设置域 ID（避免多机器人通信冲突）
export ROS_DOMAIN_ID=42

# 设置 DDS 中间件（可选）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # 或 rmw_cyclonedds_cpp
```

### 1.2 常用环境检查

```bash
ros2 doctor                    # 全面诊断 ROS2 环境
ros2 doctor --report           # 生成详细诊断报告
ros2 wtf                       # 等同于 ros2 doctor（别名）
printenv | grep ROS            # 查看所有 ROS 相关环境变量
```

---

## 2. 工作空间与包管理

### 2.1 工作空间结构

```
ros2_ws/
├── src/
│   └── my_robot_pkg/
│       ├── package.xml
│       ├── CMakeLists.txt       # C++ 包
│       ├── setup.py             # Python 包
│       ├── my_robot_pkg/
│       │   ├── __init__.py
│       │   └── collector_node.py
│       ├── launch/
│       │   └── collect.launch.py
│       ├── config/
│       │   └── params.yaml
│       ├── test/
│       │   ├── test_data_collector.py
│       │   └── CMakeLists.txt
│       └── msg/
│           └── SensorData.msg
├── build/
├── install/
└── log/
```

### 2.2 构建命令

```bash
# 构建整个工作空间
cd ~/ros2_ws && colcon build

# 构建指定包
colcon build --packages-select my_robot_pkg

# 构建并启用 symlink-install（Python 修改无需重新构建）
colcon build --symlink-install

# 构建时指定 CMake 参数
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 仅构建 C++ 包
colcon build --packages-select my_robot_pkg --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 清理构建
rm -rf build/ install/ log/
```

### 2.3 包管理

```bash
# 创建新包（Python）
ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy std_msgs sensor_msgs

# 创建新包（C++）
ros2 pkg create --build-type ament_cmake my_robot_pkg --dependencies rclcpp std_msgs sensor_msgs

# 列出所有已安装包
ros2 pkg list

# 查看包可执行文件
ros2 pkg executables my_robot_pkg

# 查看包信息
ros2 pkg info my_robot_pkg
```

---

## 3. 节点与通信核心概念

### 3.1 节点操作

```bash
# 列出所有运行中的节点
ros2 node list

# 查看节点详细信息（话题、服务、参数等）
ros2 node info /node_name

# 运行节点
ros2 run my_robot_pkg my_node

# 运行节点并重映射话题
ros2 run my_robot_pkg my_node --ros-args -r /input:=/sensors/camera

# 运行节点并设置参数
ros2 run my_robot_pkg my_node --ros-args -p rate:=30.0 -p output_file:=data.csv

# 运行节点并更改日志级别
ros2 run my_robot_pkg my_node --ros-args --log-level DEBUG
```

### 3.2 Topic（话题）

```bash
# 列出所有话题
ros2 topic list

# 列出话题及其消息类型
ros2 topic list -t

# 查看话题信息（发布者/订阅者数量、频率等）
ros2 topic info /topic_name --verbose

# 查看话题消息类型定义
ros2 interface show sensor_msgs/msg/Image

# 实时打印话题数据
ros2 topic echo /topic_name

# 限制输出条数
ros2 topic echo /topic_name --once
ros2 topic echo /topic_name --field data    # 只看特定字段

# 查看话题发布频率
ros2 topic hz /topic_name

# 查看话题带宽使用
ros2 topic bw /topic_name

# 查看话题延迟
ros2 topic delay /topic_name

# 手动发布消息
ros2 topic pub /topic_name std_msgs/msg/String "{data: 'hello'}}"
ros2 topic pub /topic_name geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --rate 10

# 查找特定类型的话题
ros2 topic find sensor_msgs/msg/LaserScan
```

### 3.3 Service（服务）

```bash
# 列出所有服务
ros2 service list

# 列出服务及类型
ros2 service list -t

# 调用服务
ros2 service call /reset std_srvs/srv/Empty
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 3.0}"

# 查看服务类型定义
ros2 interface show std_srvs/srv/Empty
```

### 3.4 Action（动作）

```bash
# 列出所有 Action
ros2 action list

# 列出 Action 及类型
ros2 action list -t

# 发送 Action 目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}}}}"

# 发送并显示反馈
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}}}}" --feedback

# 查看 Action 定义
ros2 interface show nav2_msgs/action/NavigateToPose
```

---

## 4. Topic 数据采集

### 4.1 消息类型速查

| 数据类型 | ROS2 消息类型 | 典型用途 |
|---------|--------------|---------|
| 图像 | `sensor_msgs/msg/Image` | 相机采集 |
| 点云 | `sensor_msgs/msg/PointCloud2` | LiDAR/深度相机 |
| 激光扫描 | `sensor_msgs/msg/LaserScan` | 2D LiDAR |
| IMU | `sensor_msgs/msg/Imu` | 惯性测量单元 |
| GPS/定位 | `sensor_msgs/msg/NavSatFix` | GPS 定位 |
| 里程计 | `nav_msgs/msg/Odometry` | 轮式里程计 |
| 关节状态 | `sensor_msgs/msg/JointState` | 机械臂关节 |
| 机器人状态 | `diagnostic_msgs/msg/DiagnosticArray` | 诊断信息 |
| TF 变换 | `tf2_msgs/msg/TFMessage` | 坐标变换 |
| 速度指令 | `geometry_msgs/msg/Twist` | 运动控制 |
| 占栅地图 | `nav_msgs/msg/OccupancyGrid` | 导航地图 |

### 4.2 查看消息定义

```bash
# 列出所有可用消息类型
ros2 interface list | grep msg

# 查看消息字段
ros2 interface show sensor_msgs/msg/PointCloud2

# 查看特定包的消息
ros2 interface package sensor_msgs
```

### 4.3 实时数据监控

```bash
# 监控图像话题频率（图像数据量大，不建议 echo）
ros2 topic hz /camera/image_raw

# 监控点云频率
ros2 topic hz /velodyne_points

# 查看点云带宽
ros2 topic bw /velodyne_points

# 监控多个话题频率
ros2 topic hz /camera/image_raw /lidar/points /imu/data
```

---

## 5. rosbag2 数据录制与回放

### 5.1 基础录制

```bash
# 录制所有话题（注意：数据量可能很大！）
ros2 bag record -a

# 录制指定话题
ros2 bag record /camera/image_raw /lidar/points /imu/data

# 指定输出目录和文件名
ros2 bag record -o my_dataset /camera/image_raw /lidar/points

# 录制并压缩（推荐，节省存储）
ros2 bag record -o my_dataset --compression-mode file --compression-format zstd /camera/image_raw /lidar/points

# 录制时指定存储插件（默认 mcap，也支持 sqlite3）
ros2 bag record -o my_dataset --storage mcap /topic1 /topic2

# 设置最大分片大小（大文件拆分）
ros2 bag record -o my_dataset --max-bag-size 1073741824 /topic1  # 1GB

# 设置录制时间限制（秒）
ros2 bag record -o my_dataset --duration 60 /topic1 /topic2

# 排除特定话题
ros2 bag record -a --exclude /tf /tf_static

# 限制队列大小（防止内存溢出）
ros2 bag record -o my_dataset --max-cache-size 100000000 /topic1  # 100MB 缓存
```

### 5.2 录制参数详解

```bash
# 使用参数文件配置录制
ros2 bag record \
  -o /data/robot_exp_001 \
  --storage mcap \
  --compression-mode file \
  --compression-format zstd \
  --max-bag-size 2147483648 \
  --max-cache-size 536870912 \
  /camera/color/image_raw \
  /camera/depth/image_rect_raw \
  /camera/color/camera_info \
  /lidar/points \
  /imu/data \
  /odom \
  /tf \
  /tf_static
```

### 5.3 录制信息查看

```bash
# 查看 bag 文件信息
ros2 bag info my_dataset/

# 查看详细信息（包含统计）
ros2 bag info my_dataset/ --verbose

# 输出 JSON 格式信息
ros2 bag info my_dataset/ --output-format json

# 查看特定存储格式的 bag
ros2 bag info my_dataset/ --storage mcap
```

### 5.4 回放

```bash
# 回放 bag 文件
ros2 bag play my_dataset/

# 循环回放
ros2 bag play my_dataset/ --loop

# 指定回放速率（2x 速）
ros2 bag play my_dataset/ --rate 2.0

# 0.5x 慢速回放
ros2 bag play my_dataset/ --rate 0.5

# 从特定时间开始回放（偏移秒数）
ros2 bag play my_dataset/ --start-offset 10.0

# 回放时重新映射话题
ros2 bag play my_dataset/ --remap /camera/image_raw:=/replay/camera/image_raw

# 回放时设置 QoS
ros2 bag play my_dataset/ --qos-profile-overrides-path qos_override.yaml

# 仅回放特定话题
ros2 bag play my_dataset/ --topics /camera/image_raw /odom

# 回放一次后停止（不循环）
ros2 bag play my_dataset/ --loop --read-ahead-queue-size 1000

# 等待所有订阅者就绪后开始回放
ros2 bag play my_dataset/ --wait-for-all-seconds 5
```

### 5.5 高级 rosbag2 操作

```bash
# 转换存储格式（sqlite3 → mcap）
ros2 bag convert my_dataset/ --storage-config-file convert.yaml

# convert.yaml 示例：
# output_bags:
#   - uri: my_dataset_mcap
#     storage_id: mcap
#     compression_mode: file
#     compression_format: zstd

# 过滤 bag 文件中的特定话题
ros2 bag convert input_bag/ --storage-config-file filter.yaml

# filter.yaml 示例：
# output_bags:
#   - uri: filtered_bag
#     storage_id: mcap
#     topics:
#       - /camera/image_raw
#       - /odom
```

### 5.6 Python API 录制

```python
#!/usr/bin/env python3
"""自定义数据采集节点示例"""
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterPlugin, TopicMetadata
from sensor_msgs.msg import Image, PointCloud2, Imu
from nav_msgs.msg import Odometry
import time

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # 配置 rosbag2 writer
        self.writer = SequentialWriter()
        storage_options = StorageOptions(
            uri=f'dataset_{int(time.time())}',
            storage_id='mcap'
        )
        converter_options = ConverterPlugin()
        self.writer.open(storage_options, converter_options)
        
        # 注册要录制的话题
        self._register_topic('/camera/image_raw', 'sensor_msgs/msg/Image')
        self._register_topic('/lidar/points', 'sensor_msgs/msg/PointCloud2')
        self._register_topic('/imu/data', 'sensor_msgs/msg/Imu')
        self._register_topic('/odom', 'nav_msgs/msg/Odometry')
        
        # 订阅话题
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self.create_subscription(PointCloud2, '/lidar/points', self._lidar_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        
        self.get_logger().info('数据采集节点已启动')
    
    def _register_topic(self, topic_name: str, msg_type: str):
        self.writer.create_topic(TopicMetadata(
            name=topic_name,
            type=msg_type,
            serialization_format='cdr'
        ))
    
    def _image_cb(self, msg):
        self.writer.write(
            '/camera/image_raw',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )
    
    def _lidar_cb(self, msg):
        self.writer.write(
            '/lidar/points',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )
    
    def _imu_cb(self, msg):
        self.writer.write(
            '/imu/data',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )
    
    def _odom_cb(self, msg):
        self.writer.write(
            '/odom',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )
    
    def destroy_node(self):
        del self.writer  # 关闭 bag 文件
        super().destroy_node()

def main():
    rclpy.init()
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. 传感器数据采集

### 6.1 相机数据

```bash
# 查看相机话题
ros2 topic list | grep camera

# 查看相机信息（内参、畸变等）
ros2 topic echo /camera/camera_info --once

# 查看图像话题频率
ros2 topic hz /camera/color/image_raw

# 使用 image_view 查看实时图像
ros2 run rqt_image_view rqt_image_view

# 保存单帧图像
ros2 run image_tools showimage --ros-args -i:=/camera/color/image_raw

# 使用 Python 保存图像帧
ros2 run my_pkg save_image --ros-args -p topic:=/camera/color/image_raw -p output_dir:=./images
```

### 6.2 点云数据

```bash
# 查看点云字段
ros2 topic echo /lidar/points --once | head -50

# 使用 rviz2 可视化
ros2 run rviz2 rviz2

# 点云频率与带宽
ros2 topic hz /velodyne_points
ros2 topic bw /velodyne_points

# 使用 pcl_ros 转换
ros2 run pcl_ros pointcloud_to_pcd input:=/lidar/points
```

### 6.3 LiDAR 扫描数据

```bash
# 查看 LaserScan 消息结构
ros2 interface show sensor_msgs/msg/LaserScan

# 可视化 2D 激光扫描
ros2 run rviz2 rviz2  # 添加 LaserScan 显示

# 监控扫描频率
ros2 topic hz /scan
```

### 6.4 IMU 数据

```bash
# 查看 IMU 数据
ros2 topic echo /imu/data --once

# 监控 IMU 频率（通常 100-400Hz）
ros2 topic hz /imu/data

# 查看 IMU 消息结构
ros2 interface show sensor_msgs/msg/Imu
```

### 6.5 GPS 数据

```bash
# 查看 GPS 数据
ros2 topic echo /gps/fix --once

# 查看 GPS 消息结构
ros2 interface show sensor_msgs/msg/NavSatFix

# 监控 GPS 频率
ros2 topic hz /gps/fix
```

---

## 7. 参数与配置管理

### 7.1 参数操作

```bash
# 列出节点的所有参数
ros2 param list /node_name

# 获取参数值
ros2 param get /node_name param_name

# 设置参数值
ros2 param set /node_name param_name value

# 保存节点参数到文件
ros2 param dump /node_name > params.yaml

# 从文件加载参数
ros2 param load /node_name params.yaml

# 列出参数描述
ros2 param describe /node_name param_name
```

### 7.2 参数文件格式

```yaml
# config/collector_params.yaml
/data_collector_node:
  ros__parameters:
    # 采集频率
    capture_rate: 30.0
    
    # 传感器配置
    camera_topics:
      - "/camera/color/image_raw"
      - "/camera/depth/image_rect_raw"
    lidar_topic: "/velodyne_points"
    imu_topic: "/imu/data"
    odom_topic: "/odom"
    
    # 存储配置
    output_dir: "/data/robot_experiments"
    storage_format: "mcap"
    enable_compression: true
    max_bag_size_gb: 2
    
    # 采集控制
    auto_split_duration_sec: 300  # 每5分钟自动分割
    include_tf: true
    include_diagnostics: false
```

### 7.3 YAML 参数文件使用

```bash
# 启动节点时加载参数文件
ros2 run my_pkg data_collector --ros-args --params-file config/collector_params.yaml

# 在 launch 文件中使用参数文件
ros2 launch my_pkg collect.launch.py
```

---

## 8. Launch 系统

### 8.1 Python Launch 文件示例

```python
# launch/collect_data.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, 
    RegisterShutdownHook, TimerAction, LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('dataset_name', default_value='experiment_001'),
        DeclareLaunchArgument('duration', default_value='60'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        
        # 数据采集节点
        Node(
            package='my_robot_pkg',
            executable='data_collector',
            name='data_collector',
            parameters=[PathJoinSubstitution([
                FindPackageShare('my_robot_pkg'),
                'config', 'collector_params.yaml'
            ])],
            remappings=[
                ('/camera/image_raw', '/camera/color/image_raw'),
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        
        # rosbag 录制
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', PathJoinSubstitution(['/data', LaunchConfiguration('dataset_name')]),
                '--storage', 'mcap',
                '--compression-mode', 'file',
                '--compression-format', 'zstd',
                '--max-bag-size', '2147483648',
                '/camera/color/image_raw',
                '/camera/depth/image_rect_raw',
                '/velodyne_points',
                '/imu/data',
                '/odom',
                '/tf',
                '/tf_static',
            ],
            output='screen',
        ),
        
        # TF 静态变换发布
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'camera_link'],
        ),
        
        # 定时停止录制
        TimerAction(
            period=float(LaunchConfiguration('duration')),
            actions=[
                LogInfo(msg='采集时间到，正在停止录制...'),
                # 通过 shutdown 关闭所有进程
            ],
        ),
    ])
```

### 8.2 Launch 命令

```bash
# 运行 launch 文件
ros2 launch my_robot_pkg collect_data.launch.py

# 传递参数
ros2 launch my_robot_pkg collect_data.launch.py dataset_name:=exp_002 duration:=120

# 查看 launch 文件描述
ros2 launch my_robot_pkg collect_data.launch.py --show-args

# 打印 launch 文件结构（不执行）
ros2 launch my_robot_pkg collect_data.launch.py --print
```

### 8.3 组合 Launch 文件

```python
# launch/full_system.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # 包含传感器驱动 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('camera_driver'), 'launch', 'camera.launch.py'
                ])
            ),
            launch_arguments={
                'device': '/dev/video0',
                'fps': '30',
                'resolution': '1920x1080',
            }.items(),
        ),
        
        # 包含 LiDAR 驱动 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('lidar_driver'), 'launch', 'velodyne.launch.py'
                ])
            ),
        ),
        
        # 包含数据采集 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('my_robot_pkg'), 'launch', 'collect_data.launch.py'
                ])
            ),
        ),
    ])
```

---

## 9. TF 与坐标系

### 9.1 TF 操作命令

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看 TF 树结构（生成 PDF）
ros2 run tf2_tools view_frames --ros-args -p output:=frames.pdf

# 查看特定坐标变换
ros2 run tf2_ros tf2_echo base_link camera_link

# 查看所有 TF 变换
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once

# 发布静态 TF
ros2 run tf2_ros static_transform_publisher \
  x y z yaw pitch roll frame_id child_frame_id

# 示例：相机安装在机器人上
ros2 run tf2_ros static_transform_publisher \
  0.1 0.0 0.5 0.0 0.0 0.0 base_link camera_link

# 使用四元数发布 TF
ros2 run tf2_ros static_transform_publisher \
  0.1 0.0 0.5 0.0 0.0 0.0 1.0 base_link camera_link
```

### 9.2 TF 在数据采集中的重要性

```bash
# 确保 TF 在录制中被包含
ros2 bag record /camera/image_raw /tf /tf_static /odom /lidar/points

# 验证 TF 树完整性
ros2 run tf2_tools view_frames
# 检查生成的 frames.pdf 确保所有坐标系连接正确
```

---

## 10. 日志与调试

### 10.1 日志系统

```bash
# 查看节点日志
ros2 log view

# 设置日志级别
export RCUTILS_LOG_MIN_SEVERITY=DEBUG   # DEBUG, INFO, WARN, ERROR, FATAL
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1

# 运行时设置日志级别
ros2 run my_pkg my_node --ros-args --log-level DEBUG

# 使用 rqt_console 查看日志
ros2 run rqt_console rqt_console
```

### 10.2 rqt 调试工具集

```bash
# 启动 rqt 主界面
ros2 run rqt_gui rqt_gui

# 常用 rqt 插件
ros2 run rqt_image_view rqt_image_view      # 图像查看
ros2 run rqt_graph rqt_graph                  # 节点关系图
ros2 run rqt_plot rqt_plot                    # 数据绘图
ros2 run rqt_console rqt_console              # 日志查看
ros2 run rqt_tf_tree rqt_tf_tree             # TF 树
ros2 run rqt_bag rqt_bag                      # Bag 文件可视化
```

### 10.3 话题数据绘图

```bash
# 实时绘制标量数据
ros2 run rqt_plot rqt_plot /imu/data/angular_velocity/z

# 绘制多条曲线
ros2 run rqt_plot rqt_plot /imu/data/angular_velocity/x:y:z

# 绘制位置数据
ros2 run rqt_plot rqt_plot /odom/pose/pose/position/x:y:z
```

---

## 11. 自动化测试框架

### 11.1 pytest 测试（Python）

```python
# test/test_data_collector.py
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time

@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

class TestDataCollector:
    def test_node_creation(self, ros_context):
        """测试节点能否正常创建"""
        node = Node('test_node')
        assert node.get_name() == 'test_node'
        node.destroy_node()
    
    def test_topic_publish_subscribe(self, ros_context):
        """测试话题收发"""
        node = Node('test_pub_sub')
        received = []
        
        def callback(msg):
            received.append(msg)
        
        pub = node.create_publisher(String, '/test_topic', 10)
        sub = node.create_subscription(String, '/test_topic', callback, 10)
        
        # 发送消息
        msg = String()
        msg.data = 'test_data'
        pub.publish(msg)
        
        # 等待接收
        timeout = time.time() + 2.0
        while not received and time.time() < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        assert len(received) > 0
        assert received[0].data == 'test_data'
        
        node.destroy_node()
    
    def test_camera_topic_active(self, ros_context):
        """测试相机话题是否活跃"""
        node = Node('test_camera')
        received = []
        
        def callback(msg):
            received.append(msg)
        
        sub = node.create_subscription(Image, '/camera/image_raw', callback, 10)
        
        timeout = time.time() + 5.0
        while not received and time.time() < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        assert len(received) > 0, "相机话题无数据"
        assert received[0].width > 0, "图像宽度为0"
        assert received[0].height > 0, "图像高度为0"
        
        node.destroy_node()
    
    def test_imu_frequency(self, ros_context):
        """测试 IMU 数据频率"""
        from sensor_msgs.msg import Imu
        node = Node('test_imu')
        timestamps = []
        
        def callback(msg):
            timestamps.append(time.time())
        
        sub = node.create_subscription(Imu, '/imu/data', callback, 10)
        
        # 采集 2 秒数据
        start = time.time()
        while time.time() - start < 2.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if len(timestamps) > 1:
            intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
            avg_freq = 1.0 / (sum(intervals) / len(intervals))
            assert avg_freq > 50, f"IMU 频率过低: {avg_freq:.1f}Hz"
        
        node.destroy_node()
    
    def test_tf_tree_integrity(self, ros_context):
        """测试 TF 树完整性"""
        from tf2_ros import Buffer
        node = Node('test_tf')
        buffer = Buffer()
        
        # 等待 TF 数据
        timeout = time.time() + 3.0
        while time.time() < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
                assert transform is not None
                break
            except Exception:
                continue
        
        node.destroy_node()
```

### 11.2 Launch 测试

```python
# test/test_system_launch.py
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import unittest

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    dut = launch_ros.actions.Node(
        package='my_robot_pkg',
        executable='data_collector',
        name='test_collector',
    )
    
    return launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest(),
    ])

class TestSystemReady(unittest.TestCase):
    def test_node_starts(self, proc_output, dut):
        """测试节点正常启动"""
        proc_output.assertWaitFor('数据采集节点已启动', timeout=10.0, stream='stdout')
```

### 11.3 运行测试

```bash
# 运行所有测试
colcon test

# 运行特定包的测试
colcon test --packages-select my_robot_pkg

# 运行并显示输出
colcon test --packages-select my_robot_pkg --event-handlers console_direct+

# 查看测试结果
colcon test-result --all --verbose

# 运行 pytest 直接
cd ~/ros2_ws && python -m pytest src/my_robot_pkg/test/ -v

# 运行 launch 测试
python -m pytest test/test_system_launch.py -v
```

### 11.4 集成测试脚本

```bash
#!/bin/bash
# scripts/run_integration_test.sh

set -e

echo "=== ROS2 数据采集集成测试 ==="

# 1. 检查 ROS2 环境
echo "[1/6] 检查 ROS2 环境..."
ros2 doctor --report | grep -q "All 1 checks passed" || {
    echo "ERROR: ROS2 环境检查失败"
    exit 1
}

# 2. 检查传感器话题
echo "[2/6] 检查传感器话题..."
REQUIRED_TOPICS=(
    "/camera/color/image_raw"
    "/velodyne_points"
    "/imu/data"
    "/odom"
)

for topic in "${REQUIRED_TOPICS[@]}"; do
    ros2 topic list | grep -q "$topic" || {
        echo "WARNING: 话题 $topic 未找到"
    }
done

# 3. 检查 TF 树
echo "[3/6] 检查 TF 树..."
ros2 run tf2_tools view_frames --ros-args -p output:=/tmp/test_frames.pdf
if [ -f /tmp/test_frames.pdf ]; then
    echo "TF 树已生成: /tmp/test_frames.pdf"
fi

# 4. 短时间测试录制
echo "[4/6] 测试录制 (5秒)..."
ros2 bag record -o /tmp/test_bag --duration 5 /camera/color/image_raw /odom
ros2 bag info /tmp/test_bag

# 5. 测试回放
echo "[5/6] 测试回放..."
timeout 10 ros2 bag play /tmp/test_bag &

# 6. 运行自动化测试
echo "[6/6] 运行 pytest..."
cd ~/ros2_ws && python -m pytest src/my_robot_pkg/test/ -v --tb=short

echo "=== 集成测试完成 ==="
```

---

## 12. 性能分析与监控

### 12.1 系统资源监控

```bash
# 查看 ROS2 节点资源使用
ros2 node list | while read node; do
    echo "--- $node ---"
    ros2 node info "$node" 2>/dev/null | head -20
done

# 使用 top/htop 监控
htop  # 按 F4 过滤 ros2 相关进程

# 查看 CPU 使用
ps aux | grep ros2

# 查看内存使用
pmap $(pgrep -f "ros2") | tail -1
```

### 12.2 通信性能分析

```bash
# 查看所有话题的带宽
ros2 topic list | while read topic; do
    echo -n "$topic: "
    timeout 2 ros2 topic bw "$topic" 2>/dev/null | tail -1
done

# 查看 QoS 配置
ros2 topic info /camera/image_raw --verbose

# 使用 ros2 performance_test 测试通信性能
# （需要安装 performance_test 包）
ros2 run performance_test perf_test -c ROS2 \
  -t Array1k \
  --max-runtime 30 \
  --rate 100
```

### 12.3 存储空间监控

```bash
# 监控录制文件大小
watch -n 1 'du -sh /data/dataset_*/'

# 查看磁盘使用
df -h

# 预估录制时长（根据话题带宽）
# 示例：30fps 1080p 图像 ≈ 150MB/s，1TB 可录约 110 分钟
```

---

## 13. 数据后处理与可视化

### 13.1 bag 文件转格式

```python
#!/usr/bin/env python3
"""从 bag 文件提取图像"""
import os
import cv2
import numpy as np
from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

def extract_images(bag_path, topic, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterPlugin()
    reader.open(storage_options, converter_options)
    
    bridge = CvBridge()
    count = 0
    
    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, Image)
            cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
            filename = os.path.join(output_dir, f'{count:06d}.png')
            cv2.imwrite(filename, cv_image)
            count += 1
    
    print(f'提取了 {count} 帧图像到 {output_dir}')

if __name__ == '__main__':
    extract_images('my_dataset/', '/camera/color/image_raw', './extracted_images')
```

### 13.2 点云转 PCD

```python
#!/usr/bin/env python3
"""从 bag 文件提取点云为 PCD 格式"""
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2
import rosbag2_py
from rclpy.serialization import deserialize_message

def pointcloud2_to_xyz(msg):
    """将 PointCloud2 转换为 Nx3 numpy 数组"""
    points = []
    point_step = msg.point_step
    data = msg.data
    
    for i in range(0, len(data), point_step):
        x = struct.unpack_from('f', data, i + msg.fields[0].offset)[0]
        y = struct.unpack_from('f', data, i + msg.fields[1].offset)[0]
        z = struct.unpack_from('f', data, i + msg.fields[2].offset)[0]
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            points.append([x, y, z])
    
    return np.array(points)

def save_pcd(filename, points):
    """保存为 PCD 文件"""
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""
    with open(filename, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f'{p[0]} {p[1]} {p[2]}\n')

def extract_pointclouds(bag_path, topic, output_dir):
    import os
    os.makedirs(output_dir, exist_ok=True)
    
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterPlugin()
    reader.open(storage_options, converter_options)
    
    count = 0
    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, PointCloud2)
            points = pointcloud2_to_xyz(msg)
            save_pcd(os.path.join(output_dir, f'{count:06d}.pcd'), points)
            count += 1
    
    print(f'提取了 {count} 个点云到 {output_dir}')
```

### 13.3 使用 rviz2 可视化

```bash
# 启动 rviz2
ros2 run rviz2 rviz2

# 常用显示类型：
# - Image: 相机图像
# - PointCloud2: 点云
# - LaserScan: 激光扫描
# - Map: 占栅地图
# - Pose: 位姿
# - Path: 轨迹
# - TF: 坐标系
# - RobotModel: 机器人模型
```

### 13.4 数据统计脚本

```python
#!/usr/bin/env python3
"""分析 bag 文件统计信息"""
import rosbag2_py
from rclpy.serialization import deserialize_message
import collections

def analyze_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterPlugin()
    reader.open(storage_options, converter_options)
    
    topic_stats = collections.defaultdict(lambda: {'count': 0, 'size': 0})
    
    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()
        topic_stats[topic_name]['count'] += 1
        topic_stats[topic_name]['size'] += len(data)
    
    print(f"\n{'话题':<50} {'消息数':>10} {'数据大小':>12}")
    print("-" * 80)
    
    total_size = 0
    for topic, stats in sorted(topic_stats.items()):
        size_mb = stats['size'] / (1024 * 1024)
        total_size += size_mb
        print(f"{topic:<50} {stats['count']:>10} {size_mb:>10.2f} MB")
    
    print("-" * 80)
    print(f"{'总计':<50} {'':>10} {total_size:>10.2f} MB")

if __name__ == '__main__':
    analyze_bag('my_dataset/')
```

---

import os
import cv2
import numpy as np
import pandas as pd
from scipy import stats
from PIL import Image

def robot_multimodal_clean_pipeline(raw_data_dir, output_clean_dir):
    """
    机器人多模态传感器数据全流程清洗流水线
    :param raw_data_dir: 原始采集数据根目录
    :param output_clean_dir: 清洗后数据输出目录
    """
    os.makedirs(output_clean_dir, exist_ok=True)
    # 加载全局同步后的时间戳基准表
    ts_benchmark = pd.read_csv(os.path.join(raw_data_dir, "global_timestamp.csv"))
    clean_result = []

    # 1. 视觉图像数据清洗
    img_raw_dir = os.path.join(raw_data_dir, "camera")
    img_output_dir = os.path.join(output_clean_dir, "camera")
    os.makedirs(img_output_dir, exist_ok=True)
    for img_file in os.listdir(img_raw_dir):
        img_path = os.path.join(img_raw_dir, img_file)
        try:
            # 校验图像完整性，剔除损坏文件
            img = cv2.imread(img_path)
            if img is None or img.shape[0] < 256 or img.shape[1] < 256:
                continue
            # 统一分辨率，过滤过暗过曝的无效帧
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            brightness = np.mean(gray_img)
            if brightness < 30 or brightness > 225:
                continue
            # 输出标准化图像
            cv2.imwrite(os.path.join(img_output_dir, img_file), cv2.resize(img, (640, 480)))
            clean_result.append({"type":"camera", "file":img_file, "status":"valid"})
        except Exception as e:
            print(f"图像清洗跳过无效文件: {img_file}")

    # 2. IMU时序数据清洗
    imu_raw = pd.read_csv(os.path.join(raw_data_dir, "imu_raw.csv"))
    # Z-score算法过滤加速度、角速度异常跳点
    imu_zscore = np.abs(stats.zscore(imu_raw[["acc_x","acc_y","acc_z","gyro_x","gyro_y","gyro_z"]]))
    imu_clean = imu_raw[(imu_zscore < 3).all(axis=1)]
    # 对齐全局时间戳基准
    imu_clean["timestamp"] = pd.to_numeric(imu_clean["timestamp"])
    imu_aligned = pd.merge_asof(imu_clean, ts_benchmark, on="timestamp")
    imu_aligned.to_csv(os.path.join(output_clean_dir, "imu_clean.csv"), index=False)

    # 3. 跨模态一致性校验
    static_mask = imu_clean["acc_z"].between(9.7, 9.9)
    static_img_list = [f for f in os.listdir(img_output_dir) if int(f.split(".")[0]) in imu_clean[static_mask]["timestamp"].values]
    for static_img in static_img_list:
        img = cv2.imread(os.path.join(img_output_dir, static_img))
        feature_detect = cv2.goodFeaturesToTrack(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), maxCorners=100, qualityLevel=0.01, minDistance=10)
        # 静止状态下特征点过少判定为模糊帧，直接剔除
        if feature_detect is None or len(feature_detect) < 20:
            os.remove(os.path.join(img_output_dir, static_img))

    print("机器人多模态传感器数据清洗完成，有效数据占比: {:.2f}%".format(len(clean_result)/os.listdir(img_raw_dir).__len__()*100))

# 调用示例
if __name__ == "__main__":
    robot_multimodal_clean_pipeline("./robot_raw_data_20260703", "./robot_clean_data_output")

## 14. DDS 与通信配置

### 14.1 DDS 中间件

```bash
# 查看当前使用的 DDS
echo $RMW_IMPLEMENTATION

# 切换 DDS（需要重新编译和设置环境）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 或
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 或
export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
```

### 14.2 QoS 配置

```yaml
# qos_override.yaml
/my_topic:
  reliability: reliable          # reliable | best_effort
  durability: volatile           # volatile | transient_local
  history: keep_last             # keep_last | keep_all
  depth: 10                      # 仅 keep_last 时有效
  deadline:
    sec: 1
    nsec: 0
  lifespan:
    sec: 5
    nsec: 0
```

### 14.3 多机通信

```bash
# 设置域 ID（避免冲突）
export ROS_DOMAIN_ID=42

# 设置发现等待时间
export ROS_DISCOVERY_TIMEOUT=10  # 秒

# 使用不同域 ID 隔离不同机器人
# 机器人 A: export ROS_DOMAIN_ID=1
# 机器人 B: export ROS_DOMAIN_ID=2
# 监控端: export ROS_DOMAIN_ID=1  (选择要监控的机器人)

# 使用 rmw_cyclonedds 支持多机发现
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 15. 常见数据采集场景速查

### 15.1 SLAM 数据采集

```bash
# 典型 SLAM 数据集采集
ros2 bag record -o slam_dataset \
  /camera/color/image_raw \
  /camera/depth/image_rect_raw \
  /camera/color/camera_info \
  /camera/depth/camera_info \
  /odom \
  /tf \
  /tf_static \
  --storage mcap \
  --compression-mode file \
  --compression-format zstd

# 室内建图场景
ros2 bag record -o indoor_mapping \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /map_updates \
  --storage mcap
```

### 15.2 自动驾驶数据采集

```bash
# 多传感器融合数据采集
ros2 bag record -o driving_dataset \
  /camera/front/image_raw \
  /camera/front/camera_info \
  /lidar/points \
  /radar/points \
  /imu/data \
  /gps/fix \
  /gps/vel \
  /odom \
  /tf \
  /tf_static \
  /vehicle/status \
  /vehicle/steering \
  --storage mcap \
  --compression-mode file \
  --compression-format zstd \
  --max-bag-size 4294967296  # 4GB
```

### 15.3 机械臂数据采集

```bash
# 机械臂演示数据采集
ros2 bag record -o arm_demo \
  /joint_states \
  /robot_description \
  /tf \
  /tf_static \
  /camera/color/image_raw \
  /gripper/status \
  /arm_controller/state \
  --storage mcap
```

### 15.4 巡检机器人数据采集

```bash
# 巡检场景数据采集
ros2 bag record -o patrol_data \
  /scan \
  /camera/image_raw \
  /odom \
  /tf \
  /tf_static \
  /diagnostics \
  /battery_state \
  /cmd_vel \
  /navigate_to_pose/_action/status \
  --storage mcap \
  --compression-mode file \
  --compression-format zstd
```

### 15.5 数据标注准备

```bash
# 为数据标注准备数据（仅图像 + 点云 + 标定）
ros2 bag record -o annotation_data \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /lidar/points \
  /tf \
  /tf_static \
  --storage mcap

# 提取图像帧用于标注（每秒1帧）
python3 extract_images.py \
  --bag annotation_data/ \
  --topic /camera/color/image_raw \
  --output ./frames/ \
  --sample-rate 1
```

---

## 附录 A: 常用 ROS2 消息类型参考

```bash
# 标准消息包
std_msgs          # 基础类型 (String, Int32, Float64, Bool...)
geometry_msgs     # 几何类型 (Pose, Twist, Transform, Point...)
sensor_msgs       # 传感器类型 (Image, PointCloud2, LaserScan, Imu...)
nav_msgs          # 导航类型 (Odometry, OccupancyGrid, Path...)
visualization_msgs # 可视化类型 (Marker, MarkerArray...)
tf2_msgs          # TF 变换类型
diagnostic_msgs   # 诊断类型
action_msgs       # Action 基础类型
lifecycle_msgs    # 生命周期管理

# 查看所有可用消息
ros2 interface list | grep msg

# 查看包中的消息
ros2 interface package sensor_msgs
```

## 附录 B: 常见问题排查

| 问题 | 排查命令 | 解决方案 |
|------|---------|---------|
| 话题无数据 | `ros2 topic hz /topic` | 检查节点是否启动、QoS 匹配 |
| TF 缺失 | `ros2 run tf2_tools view_frames` | 检查 TF 发布节点、坐标系名称 |
| 录制文件过大 | `ros2 bag info` + `du -sh` | 启用压缩、排除不需要的话题 |
| 回放不同步 | `ros2 bag play --rate 0.5` | 降低回放速率、检查 QoS |
| DDS 发现失败 | `ros2 doctor` | 检查网络、域 ID、防火墙 |
| 节点崩溃 | `ros2 log view` | 查看日志、检查参数配置 |
| 图像解码失败 | 检查编码格式 | 确认 `encoding` 字段 (rgb8, bgr8, mono8...) |
| 磁盘空间不足 | `df -h` | 压缩存储、定期清理旧数据 |

## 附录 C: 数据采集检查清单

- [ ] 所有传感器话题已确认活跃 (`ros2 topic list -t`)
- [ ] 各传感器频率满足要求 (`ros2 topic hz`)
- [ ] TF 树完整 (`ros2 run tf2_tools view_frames`)
- [ ] 参数文件已配置正确
- [ ] 磁盘空间充足 (`df -h`)
- [ ] 录制包含压缩选项
- [ ] 已排除不需要的高频话题
- [ ] 测试录制 5 秒并检查内容
- [ ] 标注好数据集命名规范
- [ ] 记录实验环境和配置

---

*文档生成时间: 2026-05-07*
*适用 ROS2 版本: Humble / Iron / Jazzy / Rolling*
