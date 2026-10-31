# 导入 setuptools：用于构建和安装 ROS 2 Python 包。
from setuptools import find_packages, setup

# 定义 ROS 2 Python 包的安装元信息。
setup(
    # ROS 2 包名：应与 package.xml 中的 name 一致。
    name='robopioneers_navigation',
    # 发布版本：便于后续迭代与依赖管理。
    version='0.1.0',
    # 自动发现 Python 子包。
    packages=find_packages(exclude=['test']),
    # 将 ament resource marker 安装到 ROS 2 索引目录。
    data_files=[
        # 注册包的 ament 索引标记。
        ('share/ament_index/resource_index/packages', ['resource/robopioneers_navigation']),
        # 安装 ROS 2 package.xml 元数据。
        ('share/robopioneers_navigation', ['package.xml']),
        # 安装 launch 文件，供 ros2 launch 使用。
        ('share/robopioneers_navigation/launch', ['launch/navigation.launch.py']),
        # 安装 YAML 参数文件，供 launch 加载。
        ('share/robopioneers_navigation/config', ['config/navigation.yaml']),
        # 安装联调、MCU 适配、热点组网、快速上手与暂停恢复说明。
        ('share/robopioneers_navigation/docs', [
            'docs/MCU_INTERFACE.md',
            'docs/PHONE_HOTSPOT_SETUP.md',
            'docs/QUICK_START.md',
            'docs/STOP_RESUME.md',
        ]),
    ],
    # 安装后端可执行命令所需的元信息。
    install_requires=['setuptools'],
    # 包不依赖源码目录中的额外数据文件。
    zip_safe=True,
    # 包维护者名称。
    maintainer='VOYAGER',
    # 包维护者联系地址示例。
    maintainer_email='navigation@example.com',
    # 包简述。
    description='Competition navigation state machine with two UART links and vision transport.',
    # 包许可证。
    license='MIT',
    # 安装 ROS 2 命令行程序入口。
    entry_points={
        # 注册 ros2 run robopioneers_navigation navigation_node 命令。
        'console_scripts': [
            # 主控节点入口：处理视觉、双串口和赛程状态机。
            'navigation_node = robopioneers_navigation.navigation_node:main',
        ],
    },
)
