#!/usr/bin/env python3
"""
JointLayFlatStats.py

Diagnostic-only stats script for your hexapod after laying all legs flat.

Purpose:
- Does NOT move motors.
- Does NOT enable torque.
- Reads all 18 Dynamixel AX-series motors.
- Prints tables you can paste back:
  1) Full motor status
  2) Leg-grouped view
  3) Hip/Coxa-only table
  4) Femur-only table
  5) Tibia-only table
  6) Averages by joint group
  7) Old inverted group check: MR/RR femur+tibia
  8) Copy-paste CURRENT_LAY_FLAT_POSE dictionary

Install:
    pip install dynamixel-sdk pyserial

Run:
    python JointLayFlatStats.py

Ubuntu/Raspberry Pi:
    python3 JointLayFlatStats.py
"""

import argparse
import csv
import statistics
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except ImportError:
    print("Missing library: dynamixel_sdk")
    print("Install with: pip install dynamixel-sdk pyserial")
    sys.exit(1)


BAUDRATE = 1_000_000
PROTOCOL_VERSION = 1.0
COMM_SUCCESS = 0

ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43
ADDR_MOVING = 46

RAW_PER_DEG = 1023.0 / 300.0

READ_RETRIES = 3
READ_RETRY_DELAY = 0.035

VOLT_WARN_V = 10.8
VOLT_STOP_V = 9.5
TEMP_WARN_C = 50
TEMP_STOP_C = 58
LOAD_WARN = 450
LOAD_STOP = 700


MOTOR_TO_JOINT = {
    1: "RL_hip",
    2: "FL_hip",
    3: "FR_femur",
    4: "FL_femur",
    5: "FR_tibia",
    6: "FL_tibia",
    7: "MR_hip",
    8: "ML_hip",
    9: "MR_femur",
    10: "ML_femur",
    11: "MR_tibia",
    12: "ML_tibia",
    13: "RR_hip",
    14: "FR_hip",
    15: "RR_femur",
    16: "RL_femur",
    17: "RR_tibia",
    18: "RL_tibia",
}

LEG_JOINT_IDS = {
    "FL": {"hip": 2,  "femur": 4,  "tibia": 6},
    "ML": {"hip": 8,  "femur": 10, "tibia": 12},
    "RL": {"hip": 1,  "femur": 16, "tibia": 18},
    "FR": {"hip": 14, "femur": 3,  "tibia": 5},
    "MR": {"hip": 7,  "femur": 9,  "tibia": 11},
    "RR": {"hip": 13, "femur": 15, "tibia": 17},
}

LEG_ORDER = ["FL", "ML", "RL", "FR", "MR", "RR"]
LEFT_LEGS = ["FL", "ML", "RL"]
RIGHT_LEGS = ["FR", "MR", "RR"]

# These were the old inverted/problem motors before your latest physical fix.
OLD_INVERTED_MOTOR_IDS = {9, 11, 15, 17}
OLD_INVERTED_LEGS = ["MR", "RR"]

ALL_MOTOR_IDS = sorted(MOTOR_TO_JOINT.keys())


def motor_to_leg_part(motor_id: int) -> Tuple[str, str]:
    for leg, parts in LEG_JOINT_IDS.items():
        for part, candidate_id in parts.items():
            if candidate_id == motor_id:
                return leg, part
    return "?", "?"


def decode_load_value(raw_load: Optional[int]) -> Optional[int]:
    if raw_load is None:
        return None
    if raw_load <= 1023:
        return int(raw_load)
    return int(raw_load - 1024)


def decode_load_text(raw_load: Optional[int]) -> str:
    if raw_load is None:
        return "----"
    if raw_load <= 1023:
        return f"+{raw_load}"
    return f"-{raw_load - 1024}"


def warning_text(pos, load_value, volt, temp, moving) -> str:
    warnings = []

    if pos is None:
        warnings.append("NO_REPLY")

    if volt is None:
        warnings.append("NO_VOLT")
    elif volt <= VOLT_STOP_V:
        warnings.append("VOLT_STOP")
    elif volt <= VOLT_WARN_V:
        warnings.append("LOW_VOLT")

    if temp is None:
        warnings.append("NO_TEMP")
    elif temp >= TEMP_STOP_C:
        warnings.append("TEMP_STOP")
    elif temp >= TEMP_WARN_C:
        warnings.append("TEMP_WARN")

    if load_value is None:
        warnings.append("NO_LOAD")
    elif abs(load_value) >= LOAD_STOP:
        warnings.append("LOAD_STOP")
    elif abs(load_value) >= LOAD_WARN:
        warnings.append("LOAD_WARN")

    if moving == 1:
        warnings.append("MOVING")

    return ",".join(warnings) if warnings else "OK"


def fmt(value, width=6, decimals=1):
    if value is None:
        return "----".rjust(width)
    if isinstance(value, float):
        return f"{value:.{decimals}f}".rjust(width)
    return str(value).rjust(width)


def mean_or_none(values: List[Optional[int]]) -> Optional[float]:
    clean = [int(v) for v in values if v is not None]
    if not clean:
        return None
    return float(sum(clean) / len(clean))


def stdev_or_none(values: List[Optional[int]]) -> Optional[float]:
    clean = [int(v) for v in values if v is not None]
    if len(clean) < 2:
        return None
    return float(statistics.pstdev(clean))


def minmax_or_none(values: List[Optional[int]]) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    clean = [int(v) for v in values if v is not None]
    if not clean:
        return None, None, None
    return min(clean), max(clean), max(clean) - min(clean)


def raw_to_deg_from_reference(raw: Optional[int], reference_raw: int) -> Optional[float]:
    if raw is None:
        return None
    return (int(raw) - int(reference_raw)) / RAW_PER_DEG


def detected_serial_ports() -> List[Tuple[str, str, str]]:
    if list_ports is None:
        return []

    ports = []
    for p in list_ports.comports():
        device = str(getattr(p, "device", "") or "")
        description = str(getattr(p, "description", "") or "")
        hwid = str(getattr(p, "hwid", "") or "")
        if device:
            ports.append((device, description, hwid))

    def score(item):
        dev, desc, hwid = item
        text = f"{dev} {desc} {hwid}".lower()
        preferred = any(k in text for k in ["usb", "u2d2", "ftdi", "ch340", "cp210", "acm", "serial"])
        return (0 if preferred else 1, dev)

    return sorted(ports, key=score)


def choose_serial_port() -> str:
    while True:
        ports = detected_serial_ports()

        print()
        print("===================================================")
        print(" SERIAL PORT SELECTION")
        print("===================================================")

        if ports:
            for i, (device, description, hwid) in enumerate(ports, start=1):
                print(f"{i}) {device:<22} {description}")
                if hwid:
                    print(f"   HWID: {hwid}")
        else:
            print("No serial ports auto-detected.")
            print("Common examples: COM6, /dev/ttyUSB0, /dev/ttyACM0")

        print()
        print("Type a number, exact port name, 'rescan', or press Enter for first detected port.")
        choice = input("Port choice [auto]: ").strip()

        if choice == "":
            if ports:
                selected = ports[0][0]
                print(f"Auto-selected: {selected}")
                return selected
            manual = input("Enter port manually: ").strip()
            if manual:
                return manual
            continue

        if choice.lower() in ["rescan", "scan", "refresh"]:
            continue

        if choice.isdigit() and ports:
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                selected = ports[idx][0]
                print(f"Selected: {selected}")
                return selected
            print("Invalid menu number.")
            continue

        print(f"Selected manual port: {choice}")
        return choice


class DynamixelBus:
    def __init__(self, port_name: str):
        self.port_name = port_name
        self.port_handler = PortHandler(port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

    def open(self) -> bool:
        print()
        print("===================================================")
        print(" CONNECTING")
        print("===================================================")
        print(f"Port: {self.port_name}")
        print(f"Baud: {BAUDRATE}")

        try:
            if not self.port_handler.openPort():
                print(f"FAILED: Cannot open {self.port_name}")
                return False
            if not self.port_handler.setBaudRate(BAUDRATE):
                print(f"FAILED: Cannot set baudrate {BAUDRATE}")
                return False
        except Exception as e:
            print(f"FAILED: {e}")
            print("Linux permission fix: sudo usermod -a -G dialout $USER")
            print("Then reboot.")
            return False

        print("Connected.")
        return True

    def close(self):
        try:
            self.port_handler.closePort()
        except Exception:
            pass
        print("Port closed.")

    def read1_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, int(motor_id), int(address)
            )
        except Exception:
            return None

        if result != COMM_SUCCESS or error != 0:
            return None
        return int(value)

    def read2_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, int(motor_id), int(address)
            )
        except Exception:
            return None

        if result != COMM_SUCCESS or error != 0:
            return None
        return int(value)

    def read1(self, motor_id: int, address: int) -> Optional[int]:
        for _ in range(READ_RETRIES):
            value = self.read1_once(motor_id, address)
            if value is not None:
                return value
            time.sleep(READ_RETRY_DELAY)
        return None

    def read2(self, motor_id: int, address: int) -> Optional[int]:
        for _ in range(READ_RETRIES):
            value = self.read2_once(motor_id, address)
            if value is not None:
                return value
            time.sleep(READ_RETRY_DELAY)
        return None

    def read_motor(self, motor_id: int, zero_raw: int) -> Dict[str, object]:
        joint = MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")
        leg, part = motor_to_leg_part(motor_id)

        pos = self.read2(motor_id, ADDR_PRESENT_POSITION)
        speed = self.read2(motor_id, ADDR_PRESENT_SPEED)
        raw_load = self.read2(motor_id, ADDR_PRESENT_LOAD)
        volt_raw = self.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        temp = self.read1(motor_id, ADDR_PRESENT_TEMPERATURE)
        moving = self.read1(motor_id, ADDR_MOVING)

        load_value = decode_load_value(raw_load)
        volt = (volt_raw / 10.0) if volt_raw is not None else None

        return {
            "id": motor_id,
            "joint": joint,
            "leg": leg,
            "part": part,
            "old_inverted_group": motor_id in OLD_INVERTED_MOTOR_IDS,
            "pos": pos,
            "deg_from_zero": raw_to_deg_from_reference(pos, zero_raw),
            "deg_from_512": raw_to_deg_from_reference(pos, 512),
            "deg_from_520": raw_to_deg_from_reference(pos, 520),
            "speed": speed,
            "load_raw": raw_load,
            "load_text": decode_load_text(raw_load),
            "load_value": load_value,
            "volt": volt,
            "temp": temp,
            "moving": moving,
            "warnings": warning_text(pos, load_value, volt, temp, moving),
        }


def read_all(bus: DynamixelBus, zero_raw: int) -> List[Dict[str, object]]:
    return [bus.read_motor(motor_id, zero_raw) for motor_id in ALL_MOTOR_IDS]


def rows_by_id(rows: List[Dict[str, object]]) -> Dict[int, Dict[str, object]]:
    return {int(row["id"]): row for row in rows}


def print_full_status(rows: List[Dict[str, object]], zero_raw: int):
    print()
    print("===================================================")
    print(f" FULL JOINT STATUS | zero_raw={zero_raw}")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<3} {'Part':<5} {'Inv?':<4} "
        f"{'Raw':>5} {'DegZero':>8} {'Deg520':>8} "
        f"{'Load':>7} {'Volt':>5} {'Temp':>5} {'Mov':>3} {'Warnings'}"
    )
    print("-" * 104)

    for row in rows:
        inv = "OLD" if row["old_inverted_group"] else "-"
        print(
            f"{row['id']:>2} {row['joint']:<10} {row['leg']:<3} {row['part']:<5} {inv:<4} "
            f"{fmt(row['pos'], 5, 0)} "
            f"{fmt(row['deg_from_zero'], 8, 2)} "
            f"{fmt(row['deg_from_520'], 8, 2)} "
            f"{row['load_text']:>7} "
            f"{fmt(row['volt'], 5, 1)} "
            f"{fmt(row['temp'], 5, 0)} "
            f"{fmt(row['moving'], 3, 0)} "
            f"{row['warnings']}"
        )


def print_leg_grouped(rows: List[Dict[str, object]]):
    by_id = rows_by_id(rows)

    print()
    print("===================================================")
    print(" LEG-GROUPED RAW POSITIONS")
    print("===================================================")
    print(f"{'Leg':<3} {'Hip ID':>6} {'Hip Raw':>8} {'Femur ID':>8} {'Femur Raw':>10} {'Tibia ID':>8} {'Tibia Raw':>9}")
    print("-" * 70)

    for leg in LEG_ORDER:
        ids = LEG_JOINT_IDS[leg]
        h = by_id[ids["hip"]]["pos"]
        f = by_id[ids["femur"]]["pos"]
        t = by_id[ids["tibia"]]["pos"]
        print(
            f"{leg:<3} "
            f"{ids['hip']:>6} {str(h if h is not None else '----'):>8} "
            f"{ids['femur']:>8} {str(f if f is not None else '----'):>10} "
            f"{ids['tibia']:>8} {str(t if t is not None else '----'):>9}"
        )


def print_joint_section(rows: List[Dict[str, object]], joint_part: str):
    selected = [row for row in rows if row["part"] == joint_part]
    selected = sorted(selected, key=lambda r: LEG_ORDER.index(str(r["leg"])))

    print()
    label = "HIP / COXA" if joint_part == "hip" else f"{joint_part.upper()} ONLY"
    print("===================================================")
    print(f" {label}")
    print("===================================================")
    print(f"{'Leg':<3} {'Joint':<10} {'ID':>2} {'Raw':>5} {'Deg520':>8} {'OldInv?':>7} {'Load':>7} {'Warn'}")
    print("-" * 72)

    for row in selected:
        old_inv = "YES" if row["old_inverted_group"] else "-"
        print(
            f"{row['leg']:<3} {row['joint']:<10} {row['id']:>2} "
            f"{fmt(row['pos'], 5, 0)} "
            f"{fmt(row['deg_from_520'], 8, 2)} "
            f"{old_inv:>7} "
            f"{row['load_text']:>7} "
            f"{row['warnings']}"
        )


def print_averages(rows: List[Dict[str, object]]):
    print()
    print("===================================================")
    print(" JOINT-GROUP AVERAGES")
    print("===================================================")
    print(f"{'Group':<20} {'Count':>5} {'Avg Raw':>8} {'StdDev':>8} {'Min':>6} {'Max':>6} {'Range':>7}")
    print("-" * 68)

    groups = [
        ("All hips/coxa", lambda r: r["part"] == "hip"),
        ("All femurs", lambda r: r["part"] == "femur"),
        ("All tibias", lambda r: r["part"] == "tibia"),
        ("Normal femurs", lambda r: r["part"] == "femur" and r["leg"] not in OLD_INVERTED_LEGS),
        ("Normal tibias", lambda r: r["part"] == "tibia" and r["leg"] not in OLD_INVERTED_LEGS),
        ("Old inv femurs", lambda r: r["part"] == "femur" and r["leg"] in OLD_INVERTED_LEGS),
        ("Old inv tibias", lambda r: r["part"] == "tibia" and r["leg"] in OLD_INVERTED_LEGS),
        ("Left femurs", lambda r: r["part"] == "femur" and r["leg"] in LEFT_LEGS),
        ("Right femurs", lambda r: r["part"] == "femur" and r["leg"] in RIGHT_LEGS),
        ("Left tibias", lambda r: r["part"] == "tibia" and r["leg"] in LEFT_LEGS),
        ("Right tibias", lambda r: r["part"] == "tibia" and r["leg"] in RIGHT_LEGS),
    ]

    for name, predicate in groups:
        vals = [r["pos"] for r in rows if predicate(r)]
        clean = [v for v in vals if v is not None]
        avg = mean_or_none(vals)
        sd = stdev_or_none(vals)
        mn, mx, rng = minmax_or_none(vals)
        print(
            f"{name:<20} {len(clean):>5} "
            f"{fmt(avg, 8, 2)} "
            f"{fmt(sd, 8, 2)} "
            f"{fmt(mn, 6, 0)} "
            f"{fmt(mx, 6, 0)} "
            f"{fmt(rng, 7, 0)}"
        )


def print_old_inverted_check(rows: List[Dict[str, object]]):
    by_id = rows_by_id(rows)

    print()
    print("===================================================")
    print(" OLD INVERTED MOTOR CHECK: MR/RR FEMUR + TIBIA")
    print("===================================================")
    print("Use this section after your physical inversion fix to see whether MR/RR now behave closer to the others.")
    print()
    print(f"{'Leg':<3} {'Joint':<10} {'ID':>2} {'Raw':>5} {'Deg520':>8} {'Load':>7} {'Warn'}")
    print("-" * 64)

    for motor_id in [9, 11, 15, 17]:
        row = by_id[motor_id]
        print(
            f"{row['leg']:<3} {row['joint']:<10} {row['id']:>2} "
            f"{fmt(row['pos'], 5, 0)} "
            f"{fmt(row['deg_from_520'], 8, 2)} "
            f"{row['load_text']:>7} "
            f"{row['warnings']}"
        )


def print_pose_dictionary(rows: List[Dict[str, object]]):
    by_id = rows_by_id(rows)

    print()
    print("===================================================")
    print(" COPY-PASTE CURRENT LAY-FLAT POSE DICTIONARY")
    print("===================================================")
    print("CURRENT_LAY_FLAT_POSE = {")
    for leg in LEG_ORDER:
        for part in ["hip", "femur", "tibia"]:
            motor_id = LEG_JOINT_IDS[leg][part]
            pos = by_id[motor_id]["pos"]
            pos_s = "None" if pos is None else str(pos)
            print(f"    {motor_id}: {pos_s:<4},  # {leg}_{part}")
    print("}")


def save_csv(rows: List[Dict[str, object]], csv_path: str):
    fieldnames = [
        "time", "id", "joint", "leg", "part", "old_inverted_group",
        "pos", "deg_from_zero", "deg_from_512", "deg_from_520",
        "speed", "load_raw", "load_value", "volt", "temp", "moving", "warnings",
    ]
    now = datetime.now().isoformat(timespec="seconds")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in rows:
            out = {key: row.get(key) for key in fieldnames if key != "time"}
            out["time"] = now
            writer.writerow(out)

    print(f"\nSaved CSV: {csv_path}")


def run_once(bus: DynamixelBus, zero_raw: int, csv_path: Optional[str] = None):
    rows = read_all(bus, zero_raw)

    print()
    print("===================================================")
    print(" HEXAPOD LAY-FLAT JOINT STATS SNAPSHOT")
    print("===================================================")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("Mode: diagnostic only; no torque enable; no movement command")
    print("Old inverted motors marked OLD: MR_femur=9, MR_tibia=11, RR_femur=15, RR_tibia=17")

    print_full_status(rows, zero_raw)
    print_leg_grouped(rows)
    print_joint_section(rows, "hip")
    print_joint_section(rows, "femur")
    print_joint_section(rows, "tibia")
    print_averages(rows)
    print_old_inverted_check(rows)
    print_pose_dictionary(rows)

    if csv_path:
        save_csv(rows, csv_path)


def main():
    parser = argparse.ArgumentParser(description="Read all hexapod Dynamixel joint stats while laid flat.")
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM6 or /dev/ttyUSB0. If omitted, menu is shown.")
    parser.add_argument("--zero", type=int, default=520, help="Reference raw value for DegZero column. Default: 520.")
    parser.add_argument("--watch", type=float, default=0.0, help="Refresh interval in seconds. Example: --watch 1.0")
    parser.add_argument("--csv", default=None, help="Save one snapshot to CSV path.")
    args = parser.parse_args()

    print()
    print("Diagnostic only: this script DOES NOT move motors and DOES NOT enable torque.")
    print("Lay the robot flat first, power motors on, connect serial/U2D2, then run.")
    print("Make sure no other script is using the same COM/USB port.")

    port = args.port or choose_serial_port()
    bus = DynamixelBus(port)

    if not bus.open():
        sys.exit(1)

    try:
        if args.watch and args.watch > 0:
            print("\nWatch mode. Press Ctrl+C to stop.")
            while True:
                try:
                    print("\033c", end="")
                    run_once(bus, args.zero, None)
                    time.sleep(args.watch)
                except KeyboardInterrupt:
                    print("\nStopped watch mode.")
                    break
        else:
            run_once(bus, args.zero, args.csv)

    finally:
        bus.close()


if __name__ == "__main__":
    main()
