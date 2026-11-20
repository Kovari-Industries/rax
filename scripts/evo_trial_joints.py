"""Read measured Evo arm joints from a pickup sensor log without contacting the robot."""
import argparse
import json
from pathlib import Path

SIDES = ("left", "right")


def window(path, start_ns=None, stop_ns=None):
    """Seek by receipt time so reads stay small even during a long recorded run."""
    with path.open("rb") as stream:
        stream.seek(0, 2)
        end = stream.tell()
        lo, hi = 0, end
        if start_ns is None:
            lo = max(0, end - 256_000)
        else:
            while hi - lo > 64_000:
                mid = (lo + hi) // 2
                stream.seek(mid)
                stream.readline()
                line = stream.readline()
                if not line.endswith(b"\n") or json.loads(line)["receipt_ns"] >= start_ns:
                    hi = mid
                else:
                    lo = mid
        stream.seek(lo)
        if lo:
            stream.readline()
        while stream.tell() < end:
            line = stream.readline()
            if not line.endswith(b"\n"):
                break
            row = json.loads(line)
            if stop_ns is not None and row["receipt_ns"] > stop_ns:
                break
            if row["source"] == "/joint_states":
                yield row


def joint_sample(row, capture_ns=None):
    mono = row["acquired_ns"] - (row["wall_ns"] - row["receipt_ns"])
    arms = {}
    for side in SIDES:
        indices = [row["names"].index(f"{side}_joint{i}") for i in range(1, 8)]
        arms[side] = {key: [row[src][i] if i < len(row[src]) else None for i in indices]
                      for key, src in (("q_rad", "position"), ("dq_rad_s", "velocity"), ("torque_nm", "effort"))}
    return {"joint_acquired_ns": row["acquired_ns"], "joint_monotonic_estimate_ns": mono,
            "receipt_monotonic_ns": row["receipt_ns"], "capture_minus_joint_ms":
            None if capture_ns is None else (capture_ns - mono) / 1e6, "arms": arms}


def at_capture(path, capture_ns):
    rows = window(path, capture_ns - 250_000_000, capture_ns + 250_000_000)
    samples = (joint_sample(r, capture_ns) for r in rows)
    sample = min(samples, key=lambda r: abs(r["capture_minus_joint_ms"]), default=None)
    if sample is None or abs(sample["capture_minus_joint_ms"]) > 100:
        raise ValueError("No measured joint sample within 100 ms of the wrist frame")
    return sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sensors", type=Path)
    parser.add_argument("--capture-ns", type=int)
    args = parser.parse_args()
    if args.capture_ns is not None:
        sample = at_capture(args.sensors, args.capture_ns)
    else:
        rows = list(window(args.sensors))
        if not rows:
            parser.error("No recent joint samples in the log")
        sample = joint_sample(rows[-1])
    print(json.dumps(sample, allow_nan=False))


if __name__ == "__main__":
    main()
