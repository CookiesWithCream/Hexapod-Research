"""
hexapod_joint_calibration_stats.py

Standalone Dynamixel AX-series calibration and motor-status utility for the
18-DOF hexapod project.

Purpose:
- Scan/check all expected Dynamixel IDs.
- Print every motor's present raw position, degree offset from the saved ready
  pose, goal position, voltage, temperature, load, moving state, and warnings.
- Capture a manually adjusted ready pose for hard-coding into SControlX2.py.
- Safely test individual joints with small degree offsets from the ready pose.
- Probe one joint at a time in small steps to help estimate practical physical
  limits. This is not a mathematical limit finder; always observe the robot.

Hardware assumptions:
- ROBOTIS Dynamixel AX-12A / AX-18A using Protocol 1.0.
- U2D2 / USB2DYNAMIXEL TTL adapter connected to the Dynamixel TTL bus.
- Motors are externally powered by the robot battery / CM-530 bus structure.
- Default port is COM6 and default baudrate is 1,000,000.

Safety:
- Keep the robot lifted or supported during first tests.
- Start with small movements only, for example 3 to 5 degrees.
- Be ready to cut power if a leg binds, stalls, overheats, or moves unexpectedly.
- This script uses conservative warning/stop checks, but physical observation
  is still required.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except ImportError:
    print("ERROR: dynamixel_sdk is not installed.")
    print("Install it with: pip install dynamixel-sdk")
    sys.exit(1)

# -----------------------------
# Dynamixel Protocol 1.0 setup
# -----------------------------
PROTOCOL_VERSION = 1.0
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 1_000_000

# AX-series control table addresses
ADDR_CW_ANGLE_LIMIT = 6          # 2 bytes
ADDR_CCW_ANGLE_LIMIT = 8         # 2 bytes
ADDR_TORQUE_ENABLE = 24          # 1 byte
ADDR_GOAL_POSITION = 30          # 2 bytes
ADDR_MOVING_SPEED = 32           # 2 bytes
ADDR_TORQUE_LIMIT = 34           # 2 bytes
ADDR_PRESENT_POSITION = 36       # 2 bytes
ADDR_PRESENT_LOAD = 40           # 2 bytes
ADDR_PRESENT_VOLTAGE = 42        # 1 byte, value / 10 = volts
ADDR_PRESENT_TEMPERATURE = 43    # 1 byte, Celsius
ADDR_MOVING = 46                 # 1 byte

RAW_MIN = 0
RAW_MAX = 1023
RAW_PER_DEG = 1023.0 / 300.0
DEG_PER_RAW = 300.0 / 1023.0

# Conservative safety thresholds. Adjust only if you know your hardware.
TEMP_WARN_C = 50
TEMP_STOP_C = 58
LOAD_WARN = 450
LOAD_STOP = 700
VOLT_WARN_V = 10.8
VOLT_STOP_V = 9.5
VOLT_DANGER_V = 9.2

# Slow default speed for calibration.
DEFAULT_TEST_SPEED = 80
DEFAULT_RETURN_SPEED = 100

JOINTS: Dict[int, Tuple[str, str, str]] = {
    1: ("RL_hip", "RL", "hip"),
    2: ("FL_hip", "FL", "hip"),
    3: ("FR_femur", "FR", "femur"),
    4: ("FL_femur", "FL", "femur"),
    5: ("FR_tibia", "FR", "tibia"),
    6: ("FL_tibia", "FL", "tibia"),
    7: ("MR_hip", "MR", "hip"),
    8: ("ML_hip", "ML", "hip"),
    9: ("MR_femur", "MR", "femur"),
    10: ("ML_femur", "ML", "femur"),
    11: ("MR_tibia", "MR", "tibia"),
    12: ("ML_tibia", "ML", "tibia"),
    13: ("RR_hip", "RR", "hip"),
    14: ("FR_hip", "FR", "hip"),
    15: ("RR_femur", "RR", "femur"),
    16: ("RL_femur", "RL", "femur"),
    17: ("RR_tibia", "RR", "tibia"),
    18: ("RL_tibia", "RL", "tibia"),
}

EXPECTED_IDS: List[int] = list(range(1, 19))

# Latest known ready pose from the project notes. Use snapshot command to capture
# a new one after manually positioning the robot.
READY_POSE: Dict[int, int] = {
    1: 460,    # RL_hip
    2: 747,    # FL_hip
    3: 411,    # FR_femur
    4: 366,    # FL_femur
    5: 798,    # FR_tibia
    6: 796,    # FL_tibia
    7: 608,    # MR_hip
    8: 753,    # ML_hip
    9: 627,    # MR_femur
    10: 437,   # ML_femur
    11: 216,   # MR_tibia
    12: 787,   # ML_tibia
    13: 578,   # RR_hip
    14: 575,   # FR_hip
    15: 641,   # RR_femur
    16: 412,   # RL_femur
    17: 189,   # RR_tibia
    18: 817,   # RL_tibia
}


def raw_to_deg_from_ready(motor_id: int, raw: int) -> float:
    zero = READY_POSE.get(motor_id, 512)
    return (raw - zero) * DEG_PER_RAW


def clamp_raw(raw: int) -> int:
    return max(RAW_MIN, min(RAW_MAX, int(round(raw))))


def decode_ax_load(raw_load: Optional[int]) -> Optional[int]:
    """Decode AX present load into signed approximate load value.

    AX present load uses bit 10 as direction and bits 0-9 as magnitude.
    This is an approximate internal load indicator, not calibrated torque.
    """
    if raw_load is None:
        return None
    magnitude = raw_load & 0x03FF
    direction = raw_load & 0x0400
    return -magnitude if direction else magnitude


class HexapodCalibrator:
    def __init__(self, port_name: str, baudrate: int):
        self.port_name = port_name
        self.baudrate = baudrate
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(PROTOCOL_VERSION)

    def open(self) -> None:
        if not self.port.openPort():
            raise RuntimeError(f"Failed to open port {self.port_name}")
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate {self.baudrate}")
        print(f"Connected: port={self.port_name}, baud={self.baudrate}")

    def close(self) -> None:
        try:
            self.port.closePort()
        except Exception:
            pass

    def read1(self, motor_id: int, addr: int) -> Optional[int]:
        value, comm_result, dxl_error = self.packet.read1ByteTxRx(self.port, motor_id, addr)
        if comm_result != 0 or dxl_error != 0:
            return None
        return int(value)

    def read2(self, motor_id: int, addr: int) -> Optional[int]:
        value, comm_result, dxl_error = self.packet.read2ByteTxRx(self.port, motor_id, addr)
        if comm_result != 0 or dxl_error != 0:
            return None
        return int(value)

    def write1(self, motor_id: int, addr: int, value: int) -> bool:
        comm_result, dxl_error = self.packet.write1ByteTxRx(self.port, motor_id, addr, int(value))
        return comm_result == 0 and dxl_error == 0

    def write2(self, motor_id: int, addr: int, value: int) -> bool:
        comm_result, dxl_error = self.packet.write2ByteTxRx(self.port, motor_id, addr, int(value))
        return comm_result == 0 and dxl_error == 0

    def ping(self, motor_id: int) -> bool:
        _model, comm_result, dxl_error = self.packet.ping(self.port, motor_id)
        return comm_result == 0 and dxl_error == 0

    def set_speed(self, motor_id: int, speed: int) -> None:
        self.write2(motor_id, ADDR_MOVING_SPEED, max(0, min(1023, int(speed))))

    def torque(self, motor_id: int, enable: bool) -> None:
        self.write1(motor_id, ADDR_TORQUE_ENABLE, 1 if enable else 0)

    def move_raw(self, motor_id: int, raw: int, speed: int = DEFAULT_TEST_SPEED) -> bool:
        self.set_speed(motor_id, speed)
        return self.write2(motor_id, ADDR_GOAL_POSITION, clamp_raw(raw))

    def read_status(self, motor_id: int) -> Dict[str, object]:
        name, leg, part = JOINTS.get(motor_id, (f"ID_{motor_id}", "?", "?"))
        raw_pos = self.read2(motor_id, ADDR_PRESENT_POSITION)
        goal = self.read2(motor_id, ADDR_GOAL_POSITION)
        raw_load = self.read2(motor_id, ADDR_PRESENT_LOAD)
        voltage_raw = self.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        temp = self.read1(motor_id, ADDR_PRESENT_TEMPERATURE)
        moving = self.read1(motor_id, ADDR_MOVING)
        cw_limit = self.read2(motor_id, ADDR_CW_ANGLE_LIMIT)
        ccw_limit = self.read2(motor_id, ADDR_CCW_ANGLE_LIMIT)

        load = decode_ax_load(raw_load)
        voltage = None if voltage_raw is None else voltage_raw / 10.0
        deg_zero = None if raw_pos is None else raw_to_deg_from_ready(motor_id, raw_pos)
        warnings: List[str] = []

        if raw_pos is None:
            warnings.append("NO_RESPONSE")
        if voltage is not None:
            if voltage <= VOLT_DANGER_V:
                warnings.append("VOLT_DANGER")
            elif voltage <= VOLT_STOP_V:
                warnings.append("VOLT_STOP")
            elif voltage <= VOLT_WARN_V:
                warnings.append("LOW_VOLTAGE")
        if temp is not None:
            if temp >= TEMP_STOP_C:
                warnings.append("TEMP_STOP")
            elif temp >= TEMP_WARN_C:
                warnings.append("TEMP_WARN")
        if load is not None:
            if abs(load) >= LOAD_STOP:
                warnings.append("LOAD_STOP")
            elif abs(load) >= LOAD_WARN:
                warnings.append("LOAD_WARN")

        return {
            "id": motor_id,
            "name": name,
            "leg": leg,
            "part": part,
            "raw": raw_pos,
            "deg_zero": deg_zero,
            "zero": READY_POSE.get(motor_id),
            "goal": goal,
            "load": load,
            "voltage": voltage,
            "temp": temp,
            "moving": moving,
            "cw_limit": cw_limit,
            "ccw_limit": ccw_limit,
            "warnings": warnings,
        }

    def print_status_table(self, ids: Iterable[int] = EXPECTED_IDS, save_csv: bool = False) -> List[Dict[str, object]]:
        rows = [self.read_status(mid) for mid in ids]
        print("\n" + "=" * 120)
        print(" HEXAPOD MOTOR STATUS / CALIBRATION SNAPSHOT")
        print("=" * 120)
        print(f"{'ID':>3} {'Joint':<10} {'Leg':<3} {'Part':<6} {'Raw':>5} {'DegZero':>8} {'Zero':>5} {'Goal':>5} {'Load':>7} {'Volt':>5} {'Temp':>5} {'Moving':>6}  Warnings")
        print("-" * 120)
        for r in rows:
            deg = "--" if r["deg_zero"] is None else f"{r['deg_zero']:8.2f}"
            raw = "--" if r["raw"] is None else f"{r['raw']:5d}"
            zero = "--" if r["zero"] is None else f"{r['zero']:5d}"
            goal = "--" if r["goal"] is None else f"{r['goal']:5d}"
            load = "--" if r["load"] is None else f"{r['load']:+7d}"
            volt = "--" if r["voltage"] is None else f"{r['voltage']:5.1f}"
            temp = "--" if r["temp"] is None else f"{r['temp']:5d}"
            moving = "--" if r["moving"] is None else f"{r['moving']:6d}"
            warns = "OK" if not r["warnings"] else ",".join(r["warnings"])
            print(f"{r['id']:3d} {str(r['name']):<10} {str(r['leg']):<3} {str(r['part']):<6} {raw} {deg} {zero} {goal} {load} {volt} {temp} {moving}  {warns}")
        print("-" * 120)
        connected = sum(1 for r in rows if r["raw"] is not None)
        temps = [int(r["temp"]) for r in rows if r["temp"] is not None]
        volts = [float(r["voltage"]) for r in rows if r["voltage"] is not None]
        loads = [abs(int(r["load"])) for r in rows if r["load"] is not None]
        print(f"Connected: {connected}/{len(rows)}")
        if temps:
            print(f"Max temp : {max(temps)} C")
        if volts:
            print(f"Min volt : {min(volts):.1f} V")
        if loads:
            print(f"Max load : {max(loads)}")
        print("=" * 120 + "\n")

        if save_csv:
            self.save_status_csv(rows)
        return rows

    def save_status_csv(self, rows: List[Dict[str, object]]) -> Path:
        out_dir = Path("calibration_logs")
        out_dir.mkdir(exist_ok=True)
        path = out_dir / f"motor_status_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"Saved CSV: {path}")
        return path

    def capture_ready_pose(self) -> None:
        rows = [self.read_status(mid) for mid in EXPECTED_IDS]
        print("\nCopy this READY_POSE into SControlX2.py after confirming the robot is physically in the correct stand pose:\n")
        print("READY_POSE = {")
        for r in rows:
            raw = r["raw"]
            comment = r["name"]
            if raw is None:
                print(f"    {r['id']}: None,   # {comment} NO_RESPONSE")
            else:
                print(f"    {r['id']}: {raw:4d},   # {comment}")
        print("}")

    def health_blocking(self, motor_id: Optional[int] = None) -> bool:
        ids = [motor_id] if motor_id is not None else EXPECTED_IDS
        for mid in ids:
            r = self.read_status(mid)
            if any(w in r["warnings"] for w in ["VOLT_DANGER", "VOLT_STOP", "TEMP_STOP", "LOAD_STOP", "NO_RESPONSE"]):
                print(f"SAFETY STOP on ID {mid}: {r['warnings']}")
                return True
        return False

    def test_joint(self, motor_id: int, deg: float, hold: float = 0.8, speed: int = DEFAULT_TEST_SPEED) -> None:
        if motor_id not in READY_POSE:
            raise ValueError(f"Motor ID {motor_id} has no ready-pose value")
        base = READY_POSE[motor_id]
        target = clamp_raw(base + deg * RAW_PER_DEG)
        name = JOINTS.get(motor_id, (f"ID_{motor_id}", "?", "?"))[0]
        print(f"Testing ID {motor_id} {name}: base={base}, deg={deg:+.2f}, target={target}")
        if self.health_blocking(motor_id):
            return
        self.torque(motor_id, True)
        self.move_raw(motor_id, target, speed=speed)
        time.sleep(hold)
        self.print_status_table([motor_id])
        self.move_raw(motor_id, base, speed=DEFAULT_RETURN_SPEED)
        time.sleep(hold)
        self.print_status_table([motor_id])

    def probe_joint(self, motor_id: int, direction: str, max_deg: float, step_deg: float, hold: float, speed: int) -> None:
        if direction not in {"plus", "minus"}:
            raise ValueError("direction must be 'plus' or 'minus'")
        sign = 1 if direction == "plus" else -1
        base = READY_POSE[motor_id]
        name = JOINTS.get(motor_id, (f"ID_{motor_id}", "?", "?"))[0]
        print(f"\nProbing ID {motor_id} {name} direction={direction}, max={max_deg} deg, step={step_deg} deg")
        print("Observe the physical joint. Press Ctrl+C or cut power immediately if binding/stalling occurs.")
        input("Press Enter to start this single-joint probe...")
        self.torque(motor_id, True)
        current = 0.0
        try:
            while abs(current) < abs(max_deg):
                current += sign * abs(step_deg)
                target = clamp_raw(base + current * RAW_PER_DEG)
                print(f"Step deg={current:+.2f}, target raw={target}")
                self.move_raw(motor_id, target, speed=speed)
                time.sleep(hold)
                self.print_status_table([motor_id])
                if self.health_blocking(motor_id):
                    break
                ans = input("Continue? [Enter=yes, q=stop and return]: ").strip().lower()
                if ans == "q":
                    break
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            print("Returning to ready-pose raw position...")
            self.move_raw(motor_id, base, speed=DEFAULT_RETURN_SPEED)
            time.sleep(hold)
            self.print_status_table([motor_id])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Hexapod Dynamixel calibration and motor status utility")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port, for example COM6 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Dynamixel baudrate")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="Print status for all expected motors")
    p_status_csv = sub.add_parser("status-csv", help="Print status and save a CSV log")

    sub.add_parser("snapshot", help="Print a READY_POSE dictionary from current physical positions")

    p_test = sub.add_parser("test-joint", help="Move one joint by a small degree offset from ready pose, then return")
    p_test.add_argument("id", type=int)
    p_test.add_argument("deg", type=float, help="Degree offset from ready pose, e.g. 5 or -5")
    p_test.add_argument("--hold", type=float, default=0.8)
    p_test.add_argument("--speed", type=int, default=DEFAULT_TEST_SPEED)

    p_probe = sub.add_parser("probe-joint", help="Step one joint gradually to estimate practical physical limit")
    p_probe.add_argument("id", type=int)
    p_probe.add_argument("direction", choices=["plus", "minus"])
    p_probe.add_argument("--max-deg", type=float, default=20.0)
    p_probe.add_argument("--step-deg", type=float, default=3.0)
    p_probe.add_argument("--hold", type=float, default=0.8)
    p_probe.add_argument("--speed", type=int, default=DEFAULT_TEST_SPEED)

    return parser


def main() -> None:
    args = build_parser().parse_args()
    cal = HexapodCalibrator(args.port, args.baud)
    try:
        cal.open()
        if args.cmd == "status":
            cal.print_status_table(save_csv=False)
        elif args.cmd == "status-csv":
            cal.print_status_table(save_csv=True)
        elif args.cmd == "snapshot":
            cal.print_status_table(save_csv=False)
            cal.capture_ready_pose()
        elif args.cmd == "test-joint":
            cal.test_joint(args.id, args.deg, hold=args.hold, speed=args.speed)
        elif args.cmd == "probe-joint":
            cal.probe_joint(args.id, args.direction, args.max_deg, args.step_deg, args.hold, args.speed)
    finally:
        cal.close()


if __name__ == "__main__":
    main()
