# -*- coding: utf-8 -*-
"""voyager_gate：路口「停车 → 色卡识别」闸门功能包（VOYAGER_NEW2）。

模块
----
* :mod:`voyager_gate.junction_gate`   —— 闸门判据（停稳确认 / 识别窗口 / 颜色投票）
* :mod:`voyager_gate.competition_base` —— 模板副本：路口定角转向 + STOP 定距停车
* :mod:`voyager_gate.vision_adapter`   —— VOYAGER_NEW2 增量：同进程 import nav_vision
* :mod:`voyager_gate.voyager_node`     —— 接线后的可执行节点
"""

__version__ = '2.0.0'
