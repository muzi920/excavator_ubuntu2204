"""
action_library.tasks
--------------------
高层任务生成器：单点 dig→dump 与多点循环。

直接返回标准 JSON 结构（metadata + script），可直接 dump 成 .json 文件
并被 replay_json_script.py / terminal_stepper.py 读取。
"""

from .single_dig_dump import (
    build_single_dig_dump_script,
    build_single_dig_dump_task,
)

from .multi_dig_cycle import (
    build_multi_dig_cycles,
    build_multi_dig_task,
)

__all__ = [
    "build_single_dig_dump_script",
    "build_single_dig_dump_task",
    "build_multi_dig_cycles",
    "build_multi_dig_task",
]
