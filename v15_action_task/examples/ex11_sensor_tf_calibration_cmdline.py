import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Interactive sensor TF calibration command-line tool"
    )
    parser.add_argument("--sensor-id", type=str, default="lidar_single_rear")
    parser.add_argument("--parent", type=str, default="base_link")
    parser.add_argument("--child", type=str, default="lidar_single_rear_link")
    parser.add_argument("--default-x", type=float, default=-0.5500)
    parser.add_argument("--default-y", type=float, default=-0.2000)
    parser.add_argument("--default-z", type=float, default=1.2712)
    parser.add_argument("--default-ry", type=float, default=0.0532)
    parser.add_argument("--default-pp", type=float, default=0.0349)
    parser.add_argument("--default-rr", type=float, default=3.0316)
    parser.add_argument("--step-x", type=float, default=0.001)
    parser.add_argument("--step-r", type=float, default=0.001)
    parser.add_argument(
        "--output", type=str, default="/tmp/tf_calibration_record.txt"
    )
    args = parser.parse_args()

    x = args.default_x
    y = args.default_y
    z = args.default_z
    yaw = args.default_rr
    pitch = args.default_ry
    roll = args.default_pp
    step_t = args.step_x
    step_r = args.step_r

    print(f"[ex11] sensor_id={args.sensor_id}")
    print(f"[ex11] parent={args.parent}  child={args.child}")
    print(f"[ex11] step_t={step_t} m  step_r={step_r} rad")
    print(f"[ex11] output={args.output}")
    print("[ex11] Commands: x+/x-/y+/y-/z+/z-/rz+/rz-/ry+/ry-/rx+/rx-  show  save  quit")

    try:
        while True:
            try:
                line = input("cmd> ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line == "quit":
                break
            if line == "show":
                print(
                    f"  x={x:.6f}  y={y:.6f}  z={z:.6f}  "
                    f"yaw={yaw:.6f}  pitch={pitch:.6f}  roll={roll:.6f}"
                )
                continue
            if line == "save":
                record_line = (
                    f"{x:.6f} {y:.6f} {z:.6f} "
                    f"{yaw:.6f} {pitch:.6f} {roll:.6f} "
                    f"{args.parent} {args.child}"
                )
                with open(args.output, "a", encoding="utf-8") as fh:
                    fh.write(record_line + "\n")
                print(f"  saved: {record_line}")
                continue
            if line == "x+":
                x += step_t
            elif line == "x-":
                x -= step_t
            elif line == "y+":
                y += step_t
            elif line == "y-":
                y -= step_t
            elif line == "z+":
                z += step_t
            elif line == "z-":
                z -= step_t
            elif line == "rz+":
                yaw += step_r
            elif line == "rz-":
                yaw -= step_r
            elif line == "ry+":
                pitch += step_r
            elif line == "ry-":
                pitch -= step_r
            elif line == "rx+":
                roll += step_r
            elif line == "rx-":
                roll -= step_r
            else:
                print(f"  unknown command: {line}")
                continue
            print(
                f"  -> x={x:.6f} y={y:.6f} z={z:.6f} "
                f"yaw={yaw:.6f} pitch={pitch:.6f} roll={roll:.6f}"
            )
    except KeyboardInterrupt:
        print("\n[ex11] interrupted (Ctrl-C), exiting cleanly")
        return 0

    print("[ex11] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
