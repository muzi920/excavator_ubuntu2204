#!/usr/bin/env python3
"""最小烟雾测试：感知订阅是否注册、回调是否触发。

用法：
  export ROS_DOMAIN_ID=0
  source /opt/ros/humble/setup.bash
  python3 src/shandong/v15_action_task/shandong_0.5/scripts/smoke_perception_sub.py
"""
from __future__ import annotations

import os
import sys
import threading
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_DIR = os.path.dirname(_THIS_DIR)
_SRC_DIR = os.path.dirname(os.path.dirname(_PROJECT_DIR))
for _p in (_SRC_DIR, _PROJECT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped


def main() -> int:
    rclpy.init()
    from shandong_0_5.main import V15MainNode  # type: ignore

    node: V15MainNode | None = None
    try:
        print("[smoke-percep] create node ...", flush=True)
        node = V15MainNode()
        print("[smoke-percep] node ok. node.get_name() =", node.get_name(), flush=True)

        subs = [s.topic_name for s in node.get_subscriptions_by_topic()]
        print("[smoke-percep] subscriptions =", subs, flush=True)
        has_soil = any("/perception/soil_highest_points" in s for s in subs)
        print("[smoke-percep] soil sub registered =", has_soil, flush=True)

        print("[smoke-percep] direct invoke soil_highest_points_cb with dummy msg ...", flush=True)
        dummy = PointStamped()
        dummy.header.frame_id = "smoke_test"
        dummy.point.x = 1.28
        dummy.point.y = 0.0
        dummy.point.z = 0.05
        node.soil_highest_points_cb(dummy)
        print("[smoke-percep] direct invoke returned; wait 3s for state worker ...", flush=True)
        time.sleep(3.0)
        print("[smoke-percep] state after direct invoke =", node._state, flush=True)
    except Exception as e:
        print(f"[smoke-percep] EXCEPTION: {type(e).__name__}: {e}", file=sys.stderr, flush=True)
        import traceback
        traceback.print_exc()
        return 2
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
