import json
import threading
import time


class JsonScriptReplayer:
    """按 v4 JSON 剧本格式顺序回放到 /joint_states。"""

    def __init__(
        self,
        ros_bridge,
        status_callback=None,
        *,
        feedback_mode=False,
        feedback_publish_mode="interpolate",
        tolerance_deg=1.5,
        joint_speed_deg_s=12.0,
        swing_speed_deg_s=30.0,
        fps=30.0,
        max_step_s=30.0,
        min_step_s=0.0,
        dwell_s=0.05,
    ):
        self.ros_bridge = ros_bridge
        self.status_callback = status_callback
        self.feedback_mode = bool(feedback_mode)
        self.feedback_publish_mode = str(feedback_publish_mode)
        self.tolerance_deg = float(tolerance_deg)
        self.joint_speed_deg_s = float(joint_speed_deg_s)
        self.swing_speed_deg_s = float(swing_speed_deg_s)
        self.fps = float(fps)
        self.max_step_s = float(max_step_s)
        self.min_step_s = float(min_step_s)
        self.dwell_s = float(dwell_s)
        self._stop_event = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._running = False
        self._script = []
        self._script_path = ""
        self._last_run_summary = {}

    def is_running(self):
        with self._lock:
            return self._running

    def load_script(self, script_path):
        with open(script_path, "r", encoding="utf-8") as f:
            raw = json.load(f)
        if isinstance(raw, list):
            script = raw
        elif isinstance(raw, dict) and isinstance(raw.get("script"), list):
            script = raw["script"]
        else:
            raise ValueError("JSON 剧本格式错误，根节点必须是数组，或包含 script 数组字段。")
        self._script = script
        self._script_path = script_path
        return script

    def stop(self):
        self._stop_event.set()

    def wait(self, timeout=None):
        thread = self._thread
        if thread is not None:
            thread.join(timeout=timeout)

    def start(self, script=None, script_path=None, daemon=True):
        if self.is_running():
            raise RuntimeError("剧本已经在执行中。")

        if script_path:
            script = self.load_script(script_path)
        elif script is None:
            script = self._script

        if not script:
            raise ValueError("没有可执行的剧本内容。")

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_script,
            args=(list(script),),
            daemon=daemon,
        )
        self._thread.start()
        return self._thread

    def _notify(self, info):
        if self.status_callback is not None:
            try:
                self.status_callback(info)
            except Exception:
                pass

    def _estimate_duration(self, joint_name, current_angle, target_angle, step):
        delta = abs(float(target_angle) - float(current_angle))
        ramp_up = float(step.get("ramp_up_s", 0.0) or 0.0)
        ramp_down = float(step.get("ramp_down_s", 0.0) or 0.0)

        if joint_name == "swing_yaw":
            speed_deg_s = 90.0
        else:
            speed_deg_s = 35.0

        duration = delta / speed_deg_s + ramp_up + ramp_down
        if step.get("is_init_step"):
            duration = max(duration, 1.0)
        else:
            duration = max(duration, 0.35)
        return duration

    def _feedback_speed(self, joint_name):
        if joint_name == "swing_yaw":
            return self.swing_speed_deg_s
        return self.joint_speed_deg_s

    def _run_step_feedback(self, step, current_state, *, step_index, total_steps, start_time):
        joint_name = step.get("joint")
        if joint_name not in current_state:
            return current_state, True, False

        target = float(step.get("target_val", 0.0))
        tolerance = float(step.get("tolerance_deg", self.tolerance_deg))
        speed = max(float(step.get("speed_deg_s", self._feedback_speed(joint_name))), 1e-6)
        fps = max(float(step.get("fps", self.fps)), 1e-3)
        dt = 1.0 / fps
        publish_mode = str(step.get("feedback_publish_mode", self.feedback_publish_mode))

        elapsed = time.time() - start_time
        remaining = 0.0
        self._notify(
            {
                "state": "step",
                "step_index": step_index,
                "total_steps": total_steps,
                "joint": joint_name,
                "description": step.get("description", ""),
                "target_val": target,
                "duration_s": None,
                "elapsed_s": elapsed,
                "remaining_s": remaining,
                "feedback_mode": True,
                "tolerance_deg": tolerance,
                "speed_deg_s": speed,
            }
        )

        start = time.time()
        reached = False
        timed_out = False
        while not self._stop_event.is_set():
            state = self._get_current_state()
            current = float(state.get(joint_name, current_state.get(joint_name, 0.0)))
            delta = target - current
            if abs(delta) <= tolerance:
                reached = True
                current_state[joint_name] = current
                break

            if publish_mode == "target":
                self._publish_joint_value(joint_name, target)
                current_state[joint_name] = target
            else:
                step_delta = max(-speed * dt, min(speed * dt, delta))
                next_val = current + step_delta
                self._publish_joint_value(joint_name, next_val)
                current_state[joint_name] = next_val
            time.sleep(dt)

            if self.max_step_s > 0.0 and (time.time() - start) > self.max_step_s:
                timed_out = True
                current_state[joint_name] = float(self._get_current_state().get(joint_name, current_state[joint_name]))
                break

        self._publish_joint_value(joint_name, target)
        current_state[joint_name] = float(self._get_current_state().get(joint_name, target))
        if self.min_step_s > 0.0:
            extra = self.min_step_s - (time.time() - start)
            if extra > 0.0:
                time.sleep(extra)
        if self.dwell_s > 0.0:
            time.sleep(self.dwell_s)
        return current_state, reached, timed_out

    def _get_current_state(self):
        angles = self.ros_bridge.get_v4_angles_from_joint_states_deg()
        if angles is None:
            return {
                "swing_yaw": 0.0,
                "boom_swing": 0.0,
                "arm_boom": 0.0,
                "bucket_arm": 0.0,
            }
        return {
            "swing_yaw": float(angles.get("swing_yaw", 0.0)),
            "boom_swing": float(angles.get("boom_swing", 0.0)),
            "arm_boom": float(angles.get("arm_boom", 0.0)),
            "bucket_arm": float(angles.get("bucket_arm", 0.0)),
        }

    def _publish_joint_value(self, joint_name, value):
        self.ros_bridge.publish_v4_targets_deg(**{joint_name: value})

    def _run_script(self, script):
        with self._lock:
            self._running = True
        try:
            total = len(script)
            current_state = self._get_current_state()
            start_time = time.time()
            planned_total_duration = 0.0
            estimated_durations = []
            preview_state = dict(current_state)
            for step in script:
                joint_name = step.get("joint")
                if joint_name not in preview_state:
                    estimated_durations.append(0.0)
                    continue
                target = float(step.get("target_val", 0.0))
                current = float(preview_state.get(joint_name, 0.0))
                duration = self._estimate_duration(joint_name, current, target, step)
                estimated_durations.append(duration)
                planned_total_duration += duration + 0.05
                preview_state[joint_name] = target

            self._notify(
                {
                    "state": "started",
                    "script_path": self._script_path,
                    "total_steps": total,
                    "planned_total_duration_s": planned_total_duration,
                }
            )
            for index, step in enumerate(script, start=1):
                if self._stop_event.is_set():
                    break

                joint_name = step.get("joint")
                if joint_name not in current_state:
                    continue

                target = float(step.get("target_val", 0.0))
                current = float(current_state.get(joint_name, 0.0))
                if self.feedback_mode:
                    current_state, _, _ = self._run_step_feedback(
                        step,
                        current_state,
                        step_index=index,
                        total_steps=total,
                        start_time=start_time,
                    )
                else:
                    duration = estimated_durations[index - 1]
                    fps = 30.0
                    frame_count = max(1, int(duration * fps))
                    elapsed = time.time() - start_time
                    remaining = max(
                        0.0,
                        sum(estimated_durations[index - 1:]) + max(0, total - index + 1) * 0.05
                    )

                    self._notify(
                        {
                            "state": "step",
                            "step_index": index,
                            "total_steps": total,
                            "joint": joint_name,
                            "description": step.get("description", ""),
                            "target_val": target,
                            "duration_s": duration,
                            "elapsed_s": elapsed,
                            "remaining_s": remaining,
                            "feedback_mode": False,
                        }
                    )

                    for frame in range(1, frame_count + 1):
                        if self._stop_event.is_set():
                            break
                        ratio = frame / frame_count
                        value = current + (target - current) * ratio
                        self._publish_joint_value(joint_name, value)
                        time.sleep(1.0 / fps)

                    current_state[joint_name] = target
                    self._publish_joint_value(joint_name, target)
                    time.sleep(0.05)

            elapsed_total = time.time() - start_time
            finished_normally = not self._stop_event.is_set()
            self._last_run_summary = {
                "finished_normally": finished_normally,
                "elapsed_total_s": elapsed_total,
                "total_steps": total,
            }
            self._notify(
                {
                    "state": "finished",
                    "total_steps": total,
                    "finished_normally": finished_normally,
                    "elapsed_total_s": elapsed_total,
                }
            )
        finally:
            with self._lock:
                self._running = False
