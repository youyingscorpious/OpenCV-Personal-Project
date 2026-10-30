# -*- coding: utf-8 -*-
"""方案 A 参考实现 —— 已并入可直接运行的功能包 ``voyager_gate``。

本文件保留为「算法思路的可执行入口」，不再持有实现副本，避免与生产代码分叉：

| 内容 | 位置 |
|---|---|
| 生产实现（闸门判据） | ``VOYAGER_NEW/voyager_gate/voyager_gate/junction_gate.py`` |
| 生产实现（转向 / 停车基类） | ``VOYAGER_NEW/voyager_gate/voyager_gate/competition_base.py`` |
| 接线后的可执行节点 | ``VOYAGER_NEW/voyager_gate/voyager_gate/voyager_node.py`` |
| 离线自检（6 场景 13 断言） | ``VOYAGER_NEW/voyager_gate/tools/gate_selftest.py`` |

运行::

    python3 VOYAGER_NEW/03_参考实现.py

等价于执行包内自检，不需要 ROS / 硬件 / rclpy：

* 场景 1 正常路径：行进中被拒 → 停稳确认 → 三帧同色锁定
* 场景 2 颜色抖动 1→2→1→2 不锁定
* 场景 3 停稳确认超时 + proceed → 强制开窗，不卡死
* 场景 4 停稳确认超时 + fault → 进入 FAULT
* 场景 5 闸门关闭：不再拒绝行进中的颜色，投票仍生效
* 场景 6 闸门关闭 + 投票 1 帧 = 完全退回改造前行为
"""

import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SELFTEST = os.path.join(_HERE, 'voyager_gate', 'tools', 'gate_selftest.py')


def main():
    if not os.path.isfile(_SELFTEST):
        print('未找到自检脚本：%s' % _SELFTEST)
        print('请确认 VOYAGER_NEW/voyager_gate/ 目录完整。')
        return 2
    print('实际执行：%s\n' % _SELFTEST)
    return subprocess.call([sys.executable, _SELFTEST])


if __name__ == '__main__':
    raise SystemExit(main())
