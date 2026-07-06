import sys
import time
import argparse
import csv
from datetime import datetime
from typing import Dict, Optional, Tuple, List

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
# ROBUST AUTOMATED ALL-HIP LIMIT / MOVEMENT PROBE
# ============================================================
#
# Fixes:
#   - Catches Dynamixel SDK empty packet / IndexError crashes
#   - Retries reads before declaring NO_REPLY
#   - If one hip fails, it records the error and continues
#   - Prints every movement target + feedback
#   - Prints final paste summary
#
# Run:
#   python hip_auto_probe.py --yes
#
# More visible:
#   python hip_auto_probe.py --step 5 --max-angle 75 --speed 60 --settle 0.45 --yes
#
# Test one hip only:
#   python hip_auto_probe.py --hips FL_hip --yes
#
# ============================================================


DEFAULT_PORT = "COM6"
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

TORQUE_ENABLE = 1
COMM_SUCCESS = 0

DEFAULT_SPEED = 60
DEFAULT_STEP_DEG = 5.0
DEFAULT_MAX_ANGLE = 75.0
DEFAULT_SETTLE_SECONDS = 0.45

READ_RETRIES = 3
READ_RETRY_DELAY = 0.04

CENTER_TIMEOUT_SECONDS = 3.0
CENTER_CHECK_INTERVAL = 0.12
CENTER_OK_RAW_ERROR = 8

RAW_EDGE_STOP = 8
TEMP_WARN_C = 50
TEMP_STOP_C = 58

LOAD_WARN = 450
LOAD_STOP = 700

FOLLOW_ERROR_RAW_LIMIT = 110
MIN_RAW_MOVEMENT_PER_STEP = 1
STALL_COUNT_LIMIT = 3

SESSION_READY: Dict[int, int] = {}


HIP_JOINTS = [
    "FL_hip",
    "ML_hip",
    "RL_hip",
    "FR_hip",
    "MR_hip",
    "RR_hip",
]


LEG_MOVEMENT_SIGN = {
    "FL": {"hip": 1, "femur": 1, "tibia": 1},
    "ML": {"hip": 1, "femur": 1, "tibia": 1},
    "RL": {"hip": 1, "femur": 1, "tibia": 1},
    "FR": {"hip": 1, "femur": 1, "tibia": 1},

    # Only femur/tibia are reversed for legs 5 and 6.
    # Hips stay normal here.
    "MR": {"hip": 1, "femur": -1, "tibia": -1},
    "RR": {"hip": 1, "femur": -1, "tibia": -1},
}


# ============================================================
# MAP HELPERS
# ============================================================

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


# ============================================================
# BUS
# ============================================================

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

    def write1(self, motor_id: int, address: int, value: int) -> bool:
        try:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                motor_id,
                address,
                value,
            )
        except Exception as e:
            print(f"[ID {motor_id}] WRITE1 EXCEPTION: {type(e).__name__}: {e}")
            return False

        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM ERROR: {self.packet_handler.getTxRxResult(result)}")
            return False

        if error != 0:
            print(f"[ID {motor_id}] PACKET ERROR: {self.packet_handler.getRxPacketError(error)}")
            return False

        return True

    def write2(self, motor_id: int, address: int, value: int) -> bool:
        value = int(max(0, min(1023, value)))

        try:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler,
                motor_id,
                address,
                value,
            )
        except Exception as e:
            print(f"[ID {motor_id}] WRITE2 EXCEPTION: {type(e).__name__}: {e}")
            return False

        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM ERROR: {self.packet_handler.getTxRxResult(result)}")
            return False

        if error != 0:
            print(f"[ID {motor_id}] PACKET ERROR: {self.packet_handler.getRxPacketError(error)}")
            return False

        return True

    def read1_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler,
                motor_id,
                address,
            )
        except Exception as e:
            print(f"[ID {motor_id}] READ1 EXCEPTION: {type(e).__name__}: {e}")
            return None

        if result != COMM_SUCCESS:
            return None

        if error != 0:
            return None

        return value

    def read2_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler,
                motor_id,
                address,
            )
        except Exception as e:
            print(f"[ID {motor_id}] READ2 EXCEPTION: {type(e).__name__}: {e}")
            return None

        if result != COMM_SUCCESS:
            return None

        if error != 0:
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

    def enable_torque(self, motor_id: int):
        self.write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_speed(self, motor_id: int, speed: int):
        self.write2(motor_id, ADDR_MOVING_SPEED, speed)

    def move_motor(self, motor_id: int, raw: int, speed: int) -> bool:
        self.enable_torque(motor_id)
        self.set_speed(motor_id, speed)
        time.sleep(0.015)
        return self.write2(motor_id, ADDR_GOAL_POSITION, raw)

    def move_many(self, targets: Dict[int, int], speed: int):
        for motor_id in targets:
            self.enable_torque(motor_id)
            self.set_speed(motor_id, speed)
            time.sleep(0.008)

        for motor_id, raw in targets.items():
            self.write2(motor_id, ADDR_GOAL_POSITION, raw)
            time.sleep(0.008)


# ============================================================
# FEEDBACK HELPERS
# ============================================================

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


def capture_current_pose(bus: DynamixelBus) -> Dict[int, int]:
    captured = {}

    print("\n===================================================")
    print(" CAPTURING CURRENT POSE AS SESSION_READY")
    print("===================================================")
    print("This current physical pose becomes 0 degrees.")

    for motor_id in sorted(READY_POSE.keys()):
        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)

        if pos is None:
            fallback = READY_POSE[motor_id]
            captured[motor_id] = fallback
            print(f"ID {motor_id:>2}: NO REPLY, fallback old ready {fallback}")
        else:
            captured[motor_id] = pos
            joint_name = MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")
            print(f"ID {motor_id:>2} {joint_name:<10}: {pos}")

    return captured


def session_deg_for_joint(joint_name: str, raw: int) -> float:
    motor_id = int(JOINT_LIMITS[joint_name]["id"])
    center = SESSION_READY.get(motor_id, READY_POSE[motor_id])
    direction = JOINT_DIRECTIONS.get(joint_name, 1)

    return ((raw - center) / RAW_PER_DEG) * direction


def get_motor_status(bus: DynamixelBus, motor_id: int) -> Dict[str, object]:
    joint_name = MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")
    leg_name, part_name = joint_to_leg_part(joint_name)

    raw = bus.read2(motor_id, ADDR_PRESENT_POSITION)
    speed = bus.read2(motor_id, ADDR_PRESENT_SPEED)
    load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)
    volt = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
    temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

    session_ready = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    if raw is None:
        sess_deg = None
        delta_raw = None
    else:
        sess_deg = session_deg_for_joint(joint_name, raw)
        delta_raw = raw - session_ready

    load_value = decode_load_value(load_raw)

    warnings = []

    if raw is None:
        warnings.append("NO_REPLY")
    else:
        if raw <= RAW_EDGE_STOP:
            warnings.append("RAW_EDGE_LOW")
        if raw >= 1023 - RAW_EDGE_STOP:
            warnings.append("RAW_EDGE_HIGH")

    if load_value is not None:
        if load_value >= LOAD_STOP:
            warnings.append("LOAD_STOP")
        elif load_value >= LOAD_WARN:
            warnings.append("LOAD_WARN")

    if temp is not None:
        if temp >= TEMP_STOP_C:
            warnings.append("TEMP_STOP")
        elif temp >= TEMP_WARN_C:
            warnings.append("TEMP_WARN")

    return {
        "motor_id": motor_id,
        "joint": joint_name,
        "leg": leg_name,
        "part": part_name,
        "raw": raw,
        "speed": speed,
        "session_deg": sess_deg,
        "delta_raw": delta_raw,
        "session_ready": session_ready,
        "load_raw": load_raw,
        "load_value": load_value,
        "load_text": decode_load_text(load_raw),
        "volt": volt,
        "temp": temp,
        "warnings": warnings,
    }


def print_status_table(bus: DynamixelBus):
    print("\n===================================================")
    print(" STARTING MOTOR STATUS")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Raw':>4} {'SessDeg':>8} "
        f"{'Ready':>6} {'Delta':>6} {'Load':>7} {'Volt':>5} {'Temp':>5} Warnings"
    )
    print("-" * 100)

    connected = 0

    for motor_id in sorted(READY_POSE.keys()):
        s = get_motor_status(bus, motor_id)

        if s["raw"] is None:
            print(
                f"{motor_id:>2} {s['joint']:<10} "
                f"{'----':>4} {'----':>8} {s['session_ready']:>6} {'----':>6} "
                f"{'----':>7} {'----':>5} {'----':>5} NO_REPLY"
            )
            continue

        connected += 1

        volt_text = "----" if s["volt"] is None else f"{s['volt'] / 10:.1f}"
        temp_text = "----" if s["temp"] is None else str(s["temp"])
        warn_text = "OK" if not s["warnings"] else ",".join(s["warnings"])

        print(
            f"{motor_id:>2} {s['joint']:<10} "
            f"{s['raw']:>4} {s['session_deg']:>8.2f} {s['session_ready']:>6} {s['delta_raw']:>+6} "
            f"{s['load_text']:>7} {volt_text:>5} {temp_text:>5} {warn_text}"
        )

    print("-" * 100)
    print(f"Connected: {connected}/18")


# ============================================================
# RAW CONVERSION
# ============================================================

def clamp_raw_for_joint(joint_name: str, raw: int) -> int:
    info = JOINT_LIMITS[joint_name]

    min_raw = int(info["min_raw"])
    max_raw = int(info["max_raw"])

    raw = int(max(min_raw, min(max_raw, raw)))
    raw = int(max(0, min(1023, raw)))

    return raw


def logical_offset_to_raw(joint_name: str, logical_offset_deg: float) -> Tuple[int, int, float]:
    motor_id = int(JOINT_LIMITS[joint_name]["id"])
    center = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    leg_name, part_name = joint_to_leg_part(joint_name)

    movement_sign = LEG_MOVEMENT_SIGN.get(leg_name, {}).get(part_name, 1)
    joint_direction = JOINT_DIRECTIONS.get(joint_name, 1)

    adjusted_offset = logical_offset_deg * movement_sign
    raw_unclamped = int(round(center + adjusted_offset * RAW_PER_DEG * joint_direction))
    raw_target = clamp_raw_for_joint(joint_name, raw_unclamped)

    return raw_target, raw_unclamped, adjusted_offset


# ============================================================
# RETURN TO CENTER
# ============================================================

def wait_until_motor_near(
    bus: DynamixelBus,
    motor_id: int,
    target_raw: int,
    timeout: float = CENTER_TIMEOUT_SECONDS,
    tolerance: int = CENTER_OK_RAW_ERROR,
) -> Dict[str, object]:
    start = time.time()
    last_status = get_motor_status(bus, motor_id)

    while time.time() - start < timeout:
        status = get_motor_status(bus, motor_id)
        last_status = status

        raw = status["raw"]

        if raw is not None:
            error = abs(raw - target_raw)

            if error <= tolerance:
                return status

        time.sleep(CENTER_CHECK_INTERVAL)

    return last_status


def return_joint_to_ready(bus: DynamixelBus, joint_name: str, speed: int) -> Dict[str, object]:
    motor_id = int(JOINT_LIMITS[joint_name]["id"])
    center = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    print(f"Returning {joint_name} to center raw={center}...")

    bus.move_motor(motor_id, center, speed=speed)

    status = wait_until_motor_near(
        bus=bus,
        motor_id=motor_id,
        target_raw=center,
        timeout=CENTER_TIMEOUT_SECONDS,
        tolerance=CENTER_OK_RAW_ERROR,
    )

    raw = status["raw"]
    err = None if raw is None else raw - center

    print(
        f"    center feedback raw={raw} "
        f"err={err} "
        f"deg={status['session_deg']} "
        f"load={status['load_text']} "
        f"temp={status['temp']} "
        f"warn={'OK' if not status['warnings'] else ','.join(status['warnings'])}"
    )

    return status


def return_all_hips_to_ready(bus: DynamixelBus, speed: int):
    targets = {}

    for joint_name in HIP_JOINTS:
        motor_id = int(JOINT_LIMITS[joint_name]["id"])
        targets[motor_id] = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    bus.move_many(targets, speed=speed)
    time.sleep(1.0)


# ============================================================
# STOP LOGIC
# ============================================================

def should_stop_probe(
    status: Dict[str, object],
    target_raw: int,
    previous_raw: Optional[int],
    stall_count: int,
) -> Tuple[bool, str, int]:
    raw = status["raw"]
    load_value = status["load_value"]
    temp = status["temp"]

    if raw is None:
        return True, "NO_REPLY_AFTER_RETRIES", stall_count

    if raw <= RAW_EDGE_STOP:
        return True, "RAW_EDGE_LOW", stall_count

    if raw >= 1023 - RAW_EDGE_STOP:
        return True, "RAW_EDGE_HIGH", stall_count

    if temp is not None and temp >= TEMP_STOP_C:
        return True, "TEMP_STOP", stall_count

    if load_value is not None and load_value >= LOAD_STOP:
        return True, "LOAD_STOP", stall_count

    follow_error = abs(target_raw - raw)

    if follow_error >= FOLLOW_ERROR_RAW_LIMIT:
        return True, f"FOLLOW_ERROR_RAW_{follow_error}", stall_count

    if previous_raw is not None:
        movement = abs(raw - previous_raw)

        if movement <= MIN_RAW_MOVEMENT_PER_STEP:
            stall_count += 1
        else:
            stall_count = 0

        if stall_count >= STALL_COUNT_LIMIT:
            return True, f"STALL_NO_POSITION_UPDATE_{stall_count}_STEPS", stall_count

    return False, "CONTINUE", stall_count


# ============================================================
# PROBE LOGIC
# ============================================================

def probe_one_direction(
    bus: DynamixelBus,
    joint_name: str,
    direction_label: str,
    logical_sign: int,
    step_deg: float,
    max_angle: float,
    speed: int,
    settle: float,
) -> Dict[str, object]:
    motor_id = int(JOINT_LIMITS[joint_name]["id"])
    leg_name, part_name = joint_to_leg_part(joint_name)
    center = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    print("\n===================================================")
    print(f" PROBING {joint_name} {direction_label}")
    print("===================================================")
    print(f"Motor ID      : {motor_id}")
    print(f"Leg / part    : {leg_name} / {part_name}")
    print(f"Center raw    : {center}")
    print(f"Step          : {step_deg} deg")
    print(f"Max angle     : {max_angle} deg")
    print(f"Speed         : {speed}")
    print("---------------------------------------------------")

    best_safe_logical = 0.0
    best_safe_raw = center
    stop_reason = "MAX_ANGLE_REACHED"

    previous_raw: Optional[int] = None
    stall_count = 0

    steps = int(max_angle / step_deg)
    step_logs = []

    for step_index in range(1, steps + 1):
        logical_offset = logical_sign * step_index * step_deg
        target_raw, raw_unclamped, adjusted_offset = logical_offset_to_raw(joint_name, logical_offset)

        print(
            f"\nStep {step_index:>2}/{steps} | "
            f"logical={logical_offset:+.2f}deg | "
            f"adjusted={adjusted_offset:+.2f}deg | "
            f"target_raw={target_raw} | "
            f"unclamped={raw_unclamped}"
        )

        write_ok = bus.move_motor(motor_id, target_raw, speed=speed)

        if not write_ok:
            print("    STOP: WRITE_FAILED")
            stop_reason = "WRITE_FAILED"
            break

        time.sleep(settle)

        status = get_motor_status(bus, motor_id)

        raw = status["raw"]
        sess_deg = status["session_deg"]
        load_text = status["load_text"]
        volt_text = "----" if status["volt"] is None else f"{status['volt'] / 10:.1f}"
        temp = status["temp"]
        warn_text = "OK" if not status["warnings"] else ",".join(status["warnings"])

        follow_error = None if raw is None else target_raw - raw
        actual_from_center = None if raw is None else raw - center

        print(
            f"    feedback: "
            f"raw={raw} | "
            f"sessDeg={sess_deg} | "
            f"fromCenterRaw={actual_from_center} | "
            f"followErr={follow_error} | "
            f"load={load_text} | "
            f"volt={volt_text} | "
            f"temp={temp} | "
            f"warn={warn_text}"
        )

        step_logs.append({
            "joint": joint_name,
            "direction": direction_label,
            "step_index": step_index,
            "logical_offset_deg": logical_offset,
            "adjusted_offset_deg": adjusted_offset,
            "target_raw": target_raw,
            "raw_unclamped": raw_unclamped,
            "actual_raw": raw,
            "session_deg": sess_deg,
            "from_center_raw": actual_from_center,
            "follow_error": follow_error,
            "load_text": load_text,
            "voltage": None if status["volt"] is None else status["volt"] / 10,
            "temperature": temp,
            "warnings": warn_text,
        })

        stop, reason, stall_count = should_stop_probe(
            status=status,
            target_raw=target_raw,
            previous_raw=previous_raw,
            stall_count=stall_count,
        )

        if stop:
            stop_reason = reason
            print(f"    STOP: {stop_reason}")
            break

        best_safe_logical = logical_offset
        best_safe_raw = raw
        previous_raw = raw

    print("\nDirection result:")
    print(f"Best safe {direction_label}: {best_safe_logical:+.2f} deg, raw={best_safe_raw}")
    print(f"Stop reason: {stop_reason}")

    center_status = return_joint_to_ready(bus, joint_name, speed=speed)
    center_raw = center_status["raw"]
    center_delta = None if center_raw is None else center_raw - center

    return {
        "joint": joint_name,
        "leg": leg_name,
        "part": part_name,
        "motor_id": motor_id,
        "direction": direction_label,
        "best_safe_logical_deg": best_safe_logical,
        "best_safe_raw": best_safe_raw,
        "stop_reason": stop_reason,
        "session_ready_raw": center,
        "return_raw": center_raw,
        "return_delta_raw": center_delta,
        "return_temp": center_status["temp"],
        "return_voltage": None if center_status["volt"] is None else center_status["volt"] / 10,
        "return_load": center_status["load_text"],
        "return_warnings": "OK" if not center_status["warnings"] else ",".join(center_status["warnings"]),
        "step_logs": step_logs,
    }


def probe_joint(
    bus: DynamixelBus,
    joint_name: str,
    step_deg: float,
    max_angle: float,
    speed: int,
    settle: float,
) -> List[Dict[str, object]]:
    print("\n\n###################################################")
    print(f"HIP JOINT PROBE: {joint_name}")
    print("###################################################")

    try:
        return_joint_to_ready(bus, joint_name, speed=speed)

        positive = probe_one_direction(
            bus=bus,
            joint_name=joint_name,
            direction_label="POSITIVE",
            logical_sign=1,
            step_deg=step_deg,
            max_angle=max_angle,
            speed=speed,
            settle=settle,
        )

        time.sleep(0.5)

        return_joint_to_ready(bus, joint_name, speed=speed)

        negative = probe_one_direction(
            bus=bus,
            joint_name=joint_name,
            direction_label="NEGATIVE",
            logical_sign=-1,
            step_deg=step_deg,
            max_angle=max_angle,
            speed=speed,
            settle=settle,
        )

        return_joint_to_ready(bus, joint_name, speed=speed)

        return [positive, negative]

    except Exception as e:
        motor_id = int(JOINT_LIMITS[joint_name]["id"])
        leg_name, part_name = joint_to_leg_part(joint_name)

        print(f"ERROR while probing {joint_name}: {type(e).__name__}: {e}")
        print("Trying to return this joint to ready...")

        try:
            return_joint_to_ready(bus, joint_name, speed=speed)
        except Exception:
            pass

        return [{
            "joint": joint_name,
            "leg": leg_name,
            "part": part_name,
            "motor_id": motor_id,
            "direction": "ERROR",
            "best_safe_logical_deg": 0.0,
            "best_safe_raw": SESSION_READY.get(motor_id),
            "stop_reason": f"PYTHON_EXCEPTION_{type(e).__name__}",
            "session_ready_raw": SESSION_READY.get(motor_id),
            "return_raw": None,
            "return_delta_raw": None,
            "return_temp": None,
            "return_voltage": None,
            "return_load": None,
            "return_warnings": str(e),
            "step_logs": [],
        }]


# ============================================================
# OUTPUT
# ============================================================

def save_step_csv(results: List[Dict[str, object]], filename: str):
    rows = []

    for result in results:
        for row in result.get("step_logs", []):
            rows.append(row)

    if not rows:
        return

    fieldnames = list(rows[0].keys())

    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in rows:
            writer.writerow(row)

    print(f"Step CSV saved: {filename}")


def save_summary_csv(results: List[Dict[str, object]], filename: str):
    rows = []

    for r in results:
        clean = dict(r)
        clean.pop("step_logs", None)
        rows.append(clean)

    if not rows:
        return

    fieldnames = list(rows[0].keys())

    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in rows:
            writer.writerow(row)

    print(f"Summary CSV saved: {filename}")


def print_paste_summary(results: List[Dict[str, object]]):
    print("\n\n===================================================")
    print(" PASTE THIS SUMMARY TO CHATGPT")
    print("===================================================")

    grouped: Dict[str, Dict[str, Dict[str, object]]] = {}

    for r in results:
        grouped.setdefault(r["joint"], {})
        grouped[r["direction"]] = r

    print("\nDISCOVERED HIP LIMIT SUMMARY:")

    for joint, dirs in grouped.items():
        pos = dirs.get("POSITIVE")
        neg = dirs.get("NEGATIVE")
        err = dirs.get("ERROR")

        if err:
            print(f"{joint:<10} ERROR stop={err['stop_reason']} warn={err['return_warnings']}")
            continue

        pos_deg = None if pos is None else pos["best_safe_logical_deg"]
        neg_deg = None if neg is None else neg["best_safe_logical_deg"]

        pos_reason = None if pos is None else pos["stop_reason"]
        neg_reason = None if neg is None else neg["stop_reason"]

        pos_ret = None if pos is None else pos["return_delta_raw"]
        neg_ret = None if neg is None else neg["return_delta_raw"]

        print(
            f"{joint:<10} "
            f"NEG={neg_deg} stop={neg_reason} retDelta={neg_ret} | "
            f"POS={pos_deg} stop={pos_reason} retDelta={pos_ret}"
        )

    print("\nDETAILED RESULT:")
    for r in results:
        print(
            f"{r['joint']:<10} "
            f"{r['direction']:<8} "
            f"best={r['best_safe_logical_deg']:+.2f}deg "
            f"raw={r['best_safe_raw']} "
            f"stop={r['stop_reason']} "
            f"returnDelta={r['return_delta_raw']} "
            f"temp={r['return_temp']} "
            f"volt={r['return_voltage']} "
            f"load={r['return_load']} "
            f"warn={r['return_warnings']}"
        )

    print("===================================================")


# ============================================================
# ARGS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(description="Robust automated all-hip probe with detailed feedback.")

    parser.add_argument("--port", default=DEFAULT_PORT, help="COM port. Default COM6.")

    parser.add_argument("--step", type=float, default=DEFAULT_STEP_DEG, help="Step degrees. Default 5.")
    parser.add_argument("--max-angle", type=float, default=DEFAULT_MAX_ANGLE, help="Max probe angle each side. Default 75.")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="Moving speed. Default 60.")
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_SECONDS, help="Seconds after each step. Default 0.45.")

    parser.add_argument(
        "--hips",
        nargs="*",
        default=None,
        help="Optional hip list. Example: --hips FL_hip MR_hip",
    )

    parser.add_argument("--yes", action="store_true", help="Skip initial confirmation.")

    return parser.parse_args()


# ============================================================
# MAIN
# ============================================================

def main():
    global SESSION_READY

    args = parse_args()

    selected_hips = HIP_JOINTS

    if args.hips:
        selected_hips = []

        for hip in args.hips:
            if hip not in HIP_JOINTS:
                print(f"Invalid hip joint: {hip}")
                print(f"Allowed: {HIP_JOINTS}")
                return

            selected_hips.append(hip)

    print("\n===================================================")
    print(" HEXAPOD ROBUST AUTOMATED ALL-HIP PROBE")
    print("===================================================")
    print(f"Port      : {args.port}")
    print(f"Hips      : {selected_hips}")
    print(f"Step      : {args.step} deg")
    print(f"Max angle : {args.max_angle} deg each side")
    print(f"Speed     : {args.speed}")
    print(f"Settle    : {args.settle}s")
    print("===================================================")
    print("This script runs automatically after starting.")
    print("It prints every movement position and angle.")
    print("Hold/support the robot before starting.")
    print("===================================================")
    print("Stop conditions:")
    print(f"- raw <= {RAW_EDGE_STOP} or raw >= {1023 - RAW_EDGE_STOP}")
    print(f"- temp >= {TEMP_STOP_C}C")
    print(f"- load >= {LOAD_STOP}")
    print(f"- follow error >= {FOLLOW_ERROR_RAW_LIMIT} raw")
    print(f"- repeated no position update")
    print(f"- no reply after {READ_RETRIES} retries")
    print("===================================================")

    if not args.yes:
        confirm = input("Type y to start all-hip automatic probe: ").strip().lower()

        if confirm != "y":
            print("Cancelled.")
            return

    bus = DynamixelBus(args.port)

    if not bus.open():
        return

    results: List[Dict[str, object]] = []

    try:
        SESSION_READY = capture_current_pose(bus)

        print_status_table(bus)

        print("\nReturning all hips to captured ready before starting...")
        return_all_hips_to_ready(bus, speed=args.speed)

        for hip in selected_hips:
            hip_results = probe_joint(
                bus=bus,
                joint_name=hip,
                step_deg=args.step,
                max_angle=args.max_angle,
                speed=args.speed,
                settle=args.settle,
            )

            results.extend(hip_results)

            print("\nReturning all hips to ready before next hip...")
            try:
                return_all_hips_to_ready(bus, speed=args.speed)
            except Exception as e:
                print(f"Return all hips warning: {type(e).__name__}: {e}")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        step_csv = f"hip_probe_steps_{timestamp}.csv"
        summary_csv = f"hip_probe_summary_{timestamp}.csv"

        save_step_csv(results, step_csv)
        save_summary_csv(results, summary_csv)

        print_paste_summary(results)

        print("\nDone. Paste the PASTE THIS SUMMARY section here.")

    finally:
        try:
            return_all_hips_to_ready(bus, speed=args.speed)
        except Exception:
            pass

        bus.close()


if __name__ == "__main__":
    main()