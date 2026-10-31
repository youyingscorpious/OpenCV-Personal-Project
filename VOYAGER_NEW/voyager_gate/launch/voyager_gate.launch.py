# -*- coding: utf-8 -*-
"""voyager_gate launch：启动「路口闸门导航节点」，可选再启动 PC 端视觉桥接。

视觉输入三种模式（互斥）：
  1. 默认（原方案）：板端庐山派已识别好色卡编号，AA55 帧直发到 UDP 5000；
  2. start_vision_bridge:=true：另起 PC 端 vision_bridge 节点（来自
     robopioneers_competition），通过 vision_module_path 包装 nav_vision；
  3. VOYAGER_NEW2 增量：vision_inline_enabled:=true，本节点进程内直接
     ``import nav_vision``，用 vision_module_path 指向 nav_vision.py 所在目录。
     此时不再需要 vision_bridge 节点，也不要同时跑 main.py（端口 5001/5002 冲突）。

注意
----
* 本节点已经包含 ``robopioneers_navigation`` 与 ``robopioneers_competition`` 的全部能力
  （继承链 VoyagerNode → CompetitionNode → NavigationNode）。
  **不要**再同时启动 ``navigation.launch.py`` 或 ``competition.launch.py``，
  否则会抢同一组串口（ttyUSB0/1）并同时绑定 UDP 5000。
* 节点名沿用 ``robopioneers_navigation``，话题 / 服务接口与原包完全一致。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _arg(name, default, description):
    """声明一个 launch 参数（统一转成字符串）。"""
    return DeclareLaunchArgument(
        name, default_value=str(default), description=description
    )


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _str(name):
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('voyager_gate'), 'config', 'voyager_gate.yaml']
    )

    return LaunchDescription([
        # ---- 串口 / 链路参数：默认值与原包完全一致，禁止改动 ----
        _arg('motor_port', '/dev/ttyUSB0', '电机板 UART 设备'),
        _arg('servo_port', '/dev/ttyUSB1', '外设（舵机）板 UART 设备'),
        _arg('motor_profile', 'core_crc16', '电机协议档位'),
        _arg('vision_transport', 'udp', '视觉传输：udp / tcp / serial'),
        _arg('vision_udp_bind_address', '0.0.0.0', '视觉 UDP 绑定地址'),
        _arg('vision_udp_bind_port', 5000, '视觉 UDP 绑定端口（与板端约定，勿改）'),

        # ---- 视觉桥接（默认关闭，板端已直发） ----
        _arg('start_vision_bridge', 'false',
             'true = 另起 PC 端视觉桥接节点（台架/回放用）'),
        _arg('vision_source', 'auto', 'auto / nav_vision / manual / stub'),
        _arg('vision_module_path', '', 'nav_vision.py 所在目录'),

        # ---- 转向增强总开关 ----
        _arg('route_turn_enabled', 'true',
             'false = 路口只停车不转向（完全退回原节点行为）'),

        # ---- 路口 停车 → 色卡识别 闸门（本包核心） ----
        _arg('junction_gate_enabled', 'true',
             'false = 关闸门（保留投票），配合 vote=1 即完全回到改造前'),
        _arg('junction_stop_confirm_samples', 3,
             '停稳确认所需连续 0x10 零速帧数'),
        _arg('junction_stop_confirm_dwell_s', 0.25,
             '停稳确认的持续时间下界（秒）'),
        _arg('junction_stop_confirm_timeout_s', 2.0,
             '停稳确认等待上界（秒）'),
        _arg('junction_stop_confirm_timeout_action', 'proceed',
             'proceed（强制开窗）/ fault（进 FAULT）'),
        _arg('route_color_require_after_stop', 'true',
             '只采信停稳之后的颜色帧'),
        _arg('route_color_vote_frames', 3,
             '锁定所需连续同色帧数'),
        _arg('route_color_vote_timeout_s', 0.6,
             '投票有效期（秒）'),

        # ---- VOYAGER_NEW2 增量：同进程 import nav_vision ----
        _arg('vision_inline_enabled', 'false',
             'true = 本节点进程内直接 import nav_vision，不再走 UDP 5000 结果帧；'
             'false = 原行为（板端直发编号到 5000）'),
        _arg('vision_inline_module_path', '',
             'nav_vision.py 所在目录（vision_inline_enabled=true 时必填）。'
             '例：/home/robot/ws/src/vision_nav/导航用的'),
        _arg('vision_inline_poll_period_s', 0.03,
             '轮询 nv.color()/nv.stop() 的周期（秒）'),
        _arg('vision_inline_stream_lost_fault', 5,
             'color() 连续返回 "fuck" 多少次后进 FAULT；0=只告警不 fault'),
        _arg('vision_inline_suppress_color_on_stop', 'true',
             'stop()=="1" 时屏蔽当帧 color()，防止红色 STOP 牌被当成红卡'),
        _arg('vision_inline_status_log_hz', 1.0,
             'VISION_STATUS 诊断话题发布频率（Hz）'),

        # ---- 导航节点（节点名保持 robopioneers_navigation） ----
        Node(
            package='voyager_gate',
            executable='voyager_node',
            name='robopioneers_navigation',
            output='screen',
            parameters=[
                config_file,
                {'motor_port': _str('motor_port')},
                {'servo_port': _str('servo_port')},
                {'motor_profile': _str('motor_profile')},
                {'vision_transport': _str('vision_transport')},
                {'vision_udp_bind_address': _str('vision_udp_bind_address')},
                {'vision_udp_bind_port': _int('vision_udp_bind_port')},
                {'route_turn_enabled': _bool('route_turn_enabled')},
                {'junction_gate_enabled': _bool('junction_gate_enabled')},
                {'junction_stop_confirm_samples': _int('junction_stop_confirm_samples')},
                {'junction_stop_confirm_dwell_s': _float('junction_stop_confirm_dwell_s')},
                {'junction_stop_confirm_timeout_s': _float('junction_stop_confirm_timeout_s')},
                {'junction_stop_confirm_timeout_action': _str('junction_stop_confirm_timeout_action')},
                {'route_color_require_after_stop': _bool('route_color_require_after_stop')},
                {'route_color_vote_frames': _int('route_color_vote_frames')},
                {'route_color_vote_timeout_s': _float('route_color_vote_timeout_s')},
                # VOYAGER_NEW2 增量参数
                {'vision_inline_enabled': _bool('vision_inline_enabled')},
                {'vision_module_path': _str('vision_inline_module_path')},
                {'vision_poll_period_s': _float('vision_inline_poll_period_s')},
                {'vision_stream_lost_fault': _int('vision_inline_stream_lost_fault')},
                {'vision_suppress_color_on_stop': _bool('vision_inline_suppress_color_on_stop')},
                {'vision_status_log_hz': _float('vision_inline_status_log_hz')},
            ],
        ),

        # ---- 可选：PC 端视觉桥接（复用 robopioneers_competition 的 vision_bridge） ----
        Node(
            package='robopioneers_competition',
            executable='vision_bridge',
            name='vision_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_vision_bridge')),
            parameters=[
                config_file,
                {'vision_source': _str('vision_source')},
                {'vision_module_path': _str('vision_module_path')},
            ],
        ),
    ])
