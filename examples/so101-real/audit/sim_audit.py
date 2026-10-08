"""Summarise a run's private /sim/audit: contacts with peak forces, queen pose changes, temperatures.
Usage (from the example folder):
  pixi run drawcall-robotics cat runs/<run> --topics /sim/audit | python3 audit/sim_audit.py"""
import json
import sys

events = [json.loads(json.loads(line)["msg"]["data"]) for line in sys.stdin if line.startswith("{")]
peaks = {}
for e in events:
    if e["event"] == "release":
        peaks.setdefault(e["with"], []).append(e["peakForce"])
for thing, forces in peaks.items():
    forces.sort()
    print(f"{thing}: {len(forces)} touches, peak force median {forces[len(forces) // 2]:.1f} N, max {forces[-1]:.1f} N")
prev = None
for e in events:
    if e["event"] != "heartbeat":
        continue
    q = e["queen"]
    if prev is None or abs(q["x"] - prev["x"]) + abs(q["y"] - prev["y"]) > 0.01 or abs(q["tiltDeg"] - prev["tiltDeg"]) > 10:
        print(f"t={e['time']:5.0f} s queen ({q['x']:.3f}, {q['y']:.3f}, z {q['z']:.3f}) tilt {q['tiltDeg']:3.0f} deg, "
              f"shoulder_lift {e['temperature']['shoulder_lift']:.1f} C")
        prev = q
