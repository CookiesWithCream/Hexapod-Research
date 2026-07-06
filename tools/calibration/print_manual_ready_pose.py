import sys
import time
from typing import Dict, Optional, Tuple

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except ImportError:
    print("Missing library: dynamixel_sdk")
    print("Install using:")
    print("pip install dynamixel-sdk")
    sys.exit(1)

from hexapod_kinematics import (
    READY_POSE,
    LEG_JOINTS,
    JOINT_LIMITS,
    JOINT_DIRECTIONS,
    RAW_PER_DEG,
)


# ============================================================
# PRINT MANUAL READY POSE
# ============================================================
#
# Purpose:
#   You manually place the hexapod in the physical ready pose.
#   This script reads all 18 motor positions and prints:
#       - motor ID
#       - joint name
#       - leg
#       - joint type
#       - raw position
#       - angle relative to OLD READY_POSE
#       - voltage
#       - temperature
#       - load
#
# It also prints a ready-to-copy READY_POSE dictionary.
#
# Run:
#   python print_manual_ready_pose.py
#
# This script does NOT move the robot.
# It only reads motor positions.
# ============================================================


DEFAULT_PORT = "COM6"
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 1.0

ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

COMM_SUCCESS = 0

READ_RETRIES = 3
READ_RETRY_DELAY = 0.04


def build_motor_to_joint_map() -> Dict[int, str]:
    result = {}

    for joint_name, info in JOINT_LIMITS.items():
        result[int(info["id"])] = joint_name

    return result


MOTOR_TO_JOINT = build_motor_to_joint_map()


def joint_to_leg_part(joint_name: str) -> Tuple[str, str]:
    for leg_name, parts in LEG_JOINTS.items():
        for part_name, jn in parts.items():
            if jn == joint_name:
                return leg_name, part_name

    return "?", "?"


def decode_load_value(raw_load: Optional[int]) -> Optional[int]:
    if raw_load is None:
        return None

    if raw_load <= 1023:
        return raw_load

    return raw_load - 1024


def decode_load_text(raw_load: Optional[int]) -> str:
    if raw_load is None:
        return "----"

    if raw_load <= 1023:
        return f"+{raw_load}"

    return f"-{raw_load - 1024}"


class DynamixelBus:
    def __init__(self, port_name: str):
        self.port_name = port_name
        self.port_handler = PortHandler(port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

    def open(self) -> bool:
        print("\n===================================================")
        print(" CONNECTING")
        print("===================================================")
        print(f"Port: {self.port_name}")
        print(f"Baud: {BAUDRATE}")

        if not self.port_handler.openPort():
            print(f"FAILED: Cannot open {self.port_name}")
            print("Reminder: default is COM6. If failed, check Device Manager COM port.")
            return False

        if not self.port_handler.setBaudRate(BAUDRATE):
            print(f"FAILED: Cannot set baudrate {BAUDRATE}")
            print("Reminder: check COM port / U2D2 / CM-530.")
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
                self.port_handler,
                motor_id,
                address,
            )
        except Exception:
            return None

        if result != COMM_SUCCESS or error != 0:
            return None

        return value

    def read2_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler,
                motor_id,
                address,
            )
        except Exception:
            return None

        if result != COMM_SUCCESS or error != 0:
            return None

        return value

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


def get_motor_status(bus: DynamixelBus, motor_id: int) -> Dict[str, object]:
    joint_name = MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")
    leg_name, part_name = joint_to_leg_part(joint_name)

    raw = bus.read2(motor_id, ADDR_PRESENT_POSITION)
    speed = bus.read2(motor_id, ADDR_PRESENT_SPEED)
    load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)
    voltage = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
    temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

    old_ready = READY_POSE.get(motor_id)

    if raw is not None and old_ready is not None and joint_name != "UNKNOWN":
        direction = JOINT_DIRECTIONS.get(joint_name, 1)
        deg_from_old_ready = ((raw - old_ready) / RAW_PER_DEG) * direction
        delta_raw = raw - old_ready
    else:
        deg_from_old_ready = None
        delta_raw = None

    warnings = []

    if raw is None:
        warnings.append("NO_REPLY")
    else:
        if raw <= 8:
            warnings.append("RAW_NEAR_0")
        if raw >= 1015:
            warnings.append("RAW_NEAR_1023")

    load_value = decode_load_value(load_raw)

    if load_value is not None:
        if abs(load_value) >= 700:
            warnings.append("LOAD_HIGH")
        elif abs(load_value) >= 450:
            warnings.append("LOAD_WARN")

    if temp is not None:
        if temp >= 58:
            warnings.append("TEMP_HIGH")
        elif temp >= 50:
            warnings.append("TEMP_WARN")

    return {
        "motor_id": motor_id,
        "joint_name": joint_name,
        "leg": leg_name,
        "part": part_name,
        "raw": raw,
        "old_ready": old_ready,
        "delta_raw": delta_raw,
        "deg_from_old_ready": deg_from_old_ready,
        "speed": speed,
        "load_raw": load_raw,
        "load_value": load_value,
        "load_text": decode_load_text(load_raw),
        "voltage": voltage,
        "temperature": temp,
        "warnings": warnings,
    }


def print_status_table(statuses):
    print("\n===================================================")
    print(" CURRENT MANUAL READY POSE STATUS")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<2} {'Part':<5} "
        f"{'Raw':>4} {'OldReady':>8} {'Delta':>7} {'DegFromOld':>11} "
        f"{'Load':>7} {'Volt':>5} {'Temp':>5} Warnings"
    )
    print("-" * 125)

    connected = 0

    for s in statuses:
        motor_id = s["motor_id"]

        if s["raw"] is None:
            print(
                f"{motor_id:>2} {s['joint_name']:<10} {s['leg']:<2} {s['part']:<5} "
                f"{'----':>4} {str(s['old_ready']):>8} {'----':>7} {'----':>11} "
                f"{'----':>7} {'----':>5} {'----':>5} NO_REPLY"
            )
            continue

        connected += 1

        volt_text = "----" if s["voltage"] is None else f"{s['voltage'] / 10:.1f}"
        temp_text = "----" if s["temperature"] is None else str(s["temperature"])
        deg_text = "----" if s["deg_from_old_ready"] is None else f"{s['deg_from_old_ready']:.2f}"
        delta_text = "----" if s["delta_raw"] is None else f"{s['delta_raw']:+d}"
        warn_text = "OK" if not s["warnings"] else ",".join(s["warnings"])

        print(
            f"{motor_id:>2} {s['joint_name']:<10} {s['leg']:<2} {s['part']:<5} "
            f"{s['raw']:>4} {s['old_ready']:>8} {delta_text:>7} {deg_text:>11} "
            f"{s['load_text']:>7} {volt_text:>5} {temp_text:>5} {warn_text}"
        )

    print("-" * 125)
    print(f"Connected: {connected}/18")


def print_ready_pose_dict(statuses):
    print("\n\n===================================================")
    print(" COPY THIS INTO hexapod_kinematics.py")
    print("===================================================")
    print("READY_POSE = {")

    for s in statuses:
        motor_id = s["motor_id"]
        raw = s["raw"]
        joint_name = s["joint_name"]

        if raw is None:
            print(f"    {motor_id}: {READY_POSE.get(motor_id, 512)},   # {joint_name} NO_REPLY fallback")
        else:
            print(f"    {motor_id}: {raw},   # {joint_name}")

    print("}")


def print_leg_grouped_pose(statuses):
    print("\n\n===================================================")
    print(" LEG-GROUPED SUMMARY")
    print("===================================================")

    by_joint = {s["joint_name"]: s for s in statuses}

    for leg_name in ["FL", "ML", "RL", "FR", "MR", "RR"]:
        print(f"\n[{leg_name}]")

        for part in ["hip", "femur", "tibia"]:
            joint_name = LEG_JOINTS[leg_name][part]
            s = by_joint.get(joint_name)

            if not s:
                print(f"  {part:<5} {joint_name:<10} missing")
                continue

            raw = s["raw"]
            deg = s["deg_from_old_ready"]

            raw_text = "----" if raw is None else str(raw)
            deg_text = "----" if deg is None else f"{deg:+.2f}"

            print(
                f"  {part:<5} {joint_name:<10} "
                f"ID={s['motor_id']:<2} raw={raw_text:<4} "
                f"degFromOldReady={deg_text}"
            )


def main():
    print("\n===================================================")
    print(" PRINT MANUAL READY POSE")
    print("===================================================")
    print("1. Manually place the robot into the ready pose.")
    print("2. Run this script.")
    print("3. It will NOT move the robot.")
    print("4. Copy the READY_POSE dictionary it prints.")
    print("===================================================")

    bus = DynamixelBus(DEFAULT_PORT)

    if not bus.open():
        return

    try:
        statuses = []

        for motor_id in sorted(READY_POSE.keys()):
            statuses.append(get_motor_status(bus, motor_id))

        print_status_table(statuses)
        print_leg_grouped_pose(statuses)
        print_ready_pose_dict(statuses)

        print("\nDone.")
        print("Paste the READY_POSE dictionary here if you want me to check it.")

    finally:
        bus.close()


if __name__ == "__main__":
    main()