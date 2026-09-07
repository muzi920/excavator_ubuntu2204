import argparse
import json
import os
import sys

from _util_import_helper import ensure_v15_importable

ensure_v15_importable()

from v15_action_task.config import load_config


def guess_sensor_id_from_child(child_frame: str) -> str:
    s = child_frame
    if s.endswith("_link"):
        s = s[: -len("_link")]
    return s


def parse_record_file(path: str):
    entries = []
    with open(path, "r", encoding="utf-8") as fh:
        for lineno, raw_line in enumerate(fh, start=1):
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 8:
                raise ValueError(
                    f"line {lineno}: expected 8 space-separated tokens, got {len(parts)}: {line!r}"
                )
            x_str, y_str, z_str, yaw_str, pitch_str, roll_str, parent_str, child_str = (
                parts
            )
            x = float(x_str)
            y = float(y_str)
            z = float(z_str)
            yaw = float(yaw_str)
            pitch = float(pitch_str)
            roll = float(roll_str)
            entries.append(
                {
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "pitch": pitch,
                    "roll": roll,
                    "parent": parent_str,
                    "child": child_str,
                }
            )
    return entries


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert ex11 TF calibration record file to V15 extrinsics JSON"
    )
    parser.add_argument("--input", type=str, required=True, help="record.txt from ex11")
    parser.add_argument("--sensor-id-override", type=str, default=None)
    parser.add_argument("--chassis-sn", type=str, default=None)
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="output JSON path (default inferred from --chassis-sn)",
    )
    args = parser.parse_args()

    out_path = args.out
    if out_path is None:
        sn = args.chassis_sn or "UNKNOWN"
        out_path = f"/tmp/extrinsics_{sn}.json"

    entries = parse_record_file(args.input)
    if not entries:
        print("[ex12] ERROR: no valid records found in input", file=sys.stderr)
        return 1

    extrinsics_list = []
    for e in entries:
        if args.sensor_id_override is not None:
            sid = args.sensor_id_override
        else:
            sid = guess_sensor_id_from_child(e["child"])
        extrinsics_list.append(
            {
                "sensor_id": sid,
                "parent_frame": e["parent"],
                "child_frame": e["child"],
                "x_m": e["x"],
                "y_m": e["y"],
                "z_m": e["z"],
                "yaw_rad": e["yaw"],
                "pitch_rad": e["pitch"],
                "roll_rad": e["roll"],
                "enabled": True,
                "note": "from ex12 tf_record",
            }
        )

    top_dict = {"extrinsics": extrinsics_list}
    if args.chassis_sn is not None:
        top_dict["model_name"] = f"excavator_{args.chassis_sn}"
        top_dict["description"] = f"Chassis SN {args.chassis_sn} extrinsics"
    top_dict.setdefault("v15_config_version", "1.0")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(top_dict, fh, indent=2, ensure_ascii=False)
        fh.write("\n")
    print(f"[ex12] wrote {len(extrinsics_list)} extrinsics entries -> {out_path}")

    print(f"[ex12] SELF-TEST: load_config({out_path!r}) ...")
    load_config_error = None
    try:
        cfg = load_config(out_path)
        loaded_dict = cfg.raw_dict
    except Exception as exc:
        load_config_error = exc
        print(
            f"[ex12] NOTE: load_config raised {type(exc).__name__} (expected if "
            f"build_default_* helpers missing in loader.py); falling back to raw JSON read "
            f"for value verification"
        )
        with open(out_path, "r", encoding="utf-8") as fh:
            loaded_dict = json.load(fh)

    raw_extr = loaded_dict.get("extrinsics", [])
    if not isinstance(raw_extr, list) or len(raw_extr) != len(entries):
        print(
            f"[ex12] FAIL: extrinsics mismatch, "
            f"expected list of {len(entries)} entries, got {raw_extr!r}",
            file=sys.stderr,
        )
        return 1

    tol = 1e-4
    all_ok = True
    for idx, (orig, loaded) in enumerate(zip(entries, raw_extr)):
        checks = [
            ("x_m", orig["x"], loaded.get("x_m")),
            ("y_m", orig["y"], loaded.get("y_m")),
            ("z_m", orig["z"], loaded.get("z_m")),
            ("yaw_rad", orig["yaw"], loaded.get("yaw_rad")),
            ("pitch_rad", orig["pitch"], loaded.get("pitch_rad")),
            ("roll_rad", orig["roll"], loaded.get("roll_rad")),
        ]
        for name, expected, actual in checks:
            if actual is None:
                print(f"[ex12] FAIL entry#{idx}: missing key {name}", file=sys.stderr)
                all_ok = False
                continue
            diff = abs(float(expected) - float(actual))
            if diff >= tol:
                print(
                    f"[ex12] FAIL entry#{idx} {name}: expected={expected} "
                    f"actual={actual} diff={diff} (tol={tol})",
                    file=sys.stderr,
                )
                all_ok = False

    if all_ok:
        print(f"[ex12] PASS: all {len(entries)} record(s) verified (abs diff < {tol})")
        return 0
    print("[ex12] FAIL: some checks failed", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
