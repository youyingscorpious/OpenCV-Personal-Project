# 导入 ROS 2 launch 描述对象。
from launch import LaunchDescription  # 返回可执行的 launch 配置。
# 导入 LaunchConfiguration：允许从命令行覆盖串口和协议参数。
from launch.actions import DeclareLaunchArgument  # 声明 launch 参数。
# 导入 ROS 2 节点动作。
from launch_ros.actions import Node  # 启动导航主控节点。
# 导入 ROS 包 share 目录解析器。
from launch_ros.substitutions import FindPackageShare  # 定位已安装包的数据文件。
# 导入参数替换工具。
from launch.substitutions import LaunchConfiguration  # 读取 launch 命令行值。
# 导入类型转换工具，确保命令行端口覆盖为整数参数。
from launch_ros.parameter_descriptions import ParameterValue  # 显式转换 ROS 参数类型。
# 导入路径拼接替换工具。
from launch.substitutions import PathJoinSubstitution  # 拼接已安装参数文件路径。

# 构造完整 launch 描述：默认加载双 UART 和视觉 UDP 配置。
def generate_launch_description():
    # 返回启动参数和导航节点组成的描述。
    return LaunchDescription([
        # 声明电机单片机串口设备路径。
        DeclareLaunchArgument('motor_port', default_value='/dev/ttyUSB0'),
        # 声明舵机单片机串口设备路径。
        DeclareLaunchArgument('servo_port', default_value='/dev/ttyUSB1'),
        # 声明电机协议模式，匹配第三版 CRC16 速度固件。
        DeclareLaunchArgument('motor_profile', default_value='core_crc16'),
        # 声明视觉传输方式，默认匹配 Communication_comm.py 的 UDP。
        DeclareLaunchArgument('vision_transport', default_value='udp'),
        # 声明 UDP 接收绑定地址，默认监听所有网卡。
        DeclareLaunchArgument('vision_udp_bind_address', default_value='0.0.0.0'),
        # 声明 UDP 接收端口，匹配视觉模块默认端口 5000。
        DeclareLaunchArgument('vision_udp_bind_port', default_value='5000'),
        # 创建导航主控节点。
        Node(
            # 指定 ROS 2 包名。
            package='robopioneers_navigation',
            # 指定安装后的可执行入口名称。
            executable='navigation_node',
            # 指定 ROS 图中的节点名。
            name='robopioneers_navigation',
            # 允许 launch 文件直接连接本机终端输出日志。
            output='screen',
            # 加载默认 YAML 并覆盖可由命令行调整的链路设置。
            parameters=[
                # 从已安装 share 目录读取参数文件，不依赖当前工作目录。
                PathJoinSubstitution([FindPackageShare('robopioneers_navigation'), 'config', 'navigation.yaml']),
                # 使用 launch 参数覆盖电机串口。
                {'motor_port': LaunchConfiguration('motor_port')},
                # 使用 launch 参数覆盖舵机串口。
                {'servo_port': LaunchConfiguration('servo_port')},
                # 使用 launch 参数覆盖电机 UART 协议模式。
                {'motor_profile': LaunchConfiguration('motor_profile')},
                # 使用 launch 参数覆盖视觉链路传输方式。
                {'vision_transport': LaunchConfiguration('vision_transport')},
                # 使用 launch 参数覆盖 UDP 监听地址。
                {'vision_udp_bind_address': LaunchConfiguration('vision_udp_bind_address')},
                # 使用 launch 参数覆盖 UDP 监听端口。
                {'vision_udp_bind_port': ParameterValue(LaunchConfiguration('vision_udp_bind_port'), value_type=int)},
            ],
        ),
    ])
