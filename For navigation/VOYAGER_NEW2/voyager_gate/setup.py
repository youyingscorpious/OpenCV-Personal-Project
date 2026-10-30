# -*- coding: utf-8 -*-
# ament_python 构建入口：注册 1 个可执行节点。
from glob import glob  # 用于收集 config / launch / tools 下的数据文件。

from setuptools import setup  # ament_python 使用标准 setuptools。

# 包名必须与 package.xml 中的 <name> 完全一致。
package_name = 'voyager_gate'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # 注册 ament 资源索引路径。
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装 package.xml 供 ros2 pkg 查询。
        ('share/' + package_name, ['package.xml']),
        # 安装参数文件和 launch 文件。
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # 安装说明与自检脚本，方便拷到车上直接看 / 直接跑。
        ('share/' + package_name + '/docs', glob('*.md')),
        ('share/' + package_name + '/tools', glob('tools/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robopioneers Navigation Team',
    maintainer_email='team@example.com',
    description='Robopioneers 路口「停车 → 色卡识别」闸门（方案 A：纯导航端，零协议改动）',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 唯一可执行入口：在 CompetitionNode 之上挂载路口闸门。
            'voyager_node = voyager_gate.voyager_node:main',
        ],
    },
)
