import math


def build_dump_point(task_config):
    if "dump_point" in task_config:
        point = task_config["dump_point"]
        return {
            "x": float(point["x"]),
            "y": float(point["y"]),
            "z": float(point["z"]),
            "source": "task_config.dump_point",
        }

    strategy = task_config.get("dump_strategy")
    if not isinstance(strategy, dict):
        raise ValueError("task-config 必须提供 dump_point 或 dump_strategy。")

    direction = str(strategy.get("direction", "left")).lower()
    yaw_deg = float(strategy["yaw_deg"])
    yaw_deg = -abs(yaw_deg) if direction == "right" else abs(yaw_deg)

    radius = float(strategy["dump_radius"])
    height = float(strategy["dump_height"])
    yaw_rad = math.radians(yaw_deg)
    return {
        "x": radius * math.cos(yaw_rad),
        "y": radius * math.sin(yaw_rad),
        "z": height,
        "yaw_deg": yaw_deg,
        "source": "task_config.dump_strategy",
    }
