#!/usr/bin/env python3
"""
AverageVsCalibratedFlatPose.py

Move-only flat pose script for the hexapod.

This corrected version has only the useful flat calibration modes:

1) average_perfect
   Purpose:
   - Clean "perfect average" pose.
   - Same value for every similar joint.
   - No mechanical offsets.
   - Good for showing code-balanced / mathematical reference.

   Defaults:
   - All hips/coxa = 520
   - All femurs = 507   # rounded from new femur average 506.5
   - All tibias = 363   # rounded from new tibia average 362.83

2) calibrated_physical
   Purpose:
   - Real physically flat pose after calibration.
   - Keeps normal hips at 520 but MR_hip at 575.
   - Applies per-leg femur/tibia offsets from the latest lay-flat reading.

   Defaults:
   - FL femur 456, tibia 364
   - ML femur 529, tibia 364
   - RL femur 529, tibia 357
   - FR femur 513, tibia 364
   - MR femur 505, tibia 367
   - RR femur 507, tibia 361
   - MR hip 575, all other hips 520

3) compare
   - Moves to average_perfect first.
   - Waits for Enter.
   - Moves to calibrated_physical.
   - Useful for visually comparing code-perfect vs physical-perfect calibration.

Install:
    pip install dynamixel-sdk pyserial

Run:
    python AverageVsCalibratedFlatPose.py

Ubuntu/Raspberry Pi:
    python3 AverageVsCalibratedFlatPose.py

Examples:
    python3 AverageVsCalibratedFlatPose.py --mode average_perfect
    python3 AverageVsCalibratedFlatPose.py --mode calibrated_physical
    python3 AverageVsCalibratedFlatPose.py --mode compare
"""

import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

try:
    from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite
except ImportError:
    print("Missing library: dynamixel_sdk")
    print("Install with: pip install dynamixel-sdk pyserial")
    sys.exit(1)


# ============================================================
# DYNAMIXEL AX / PROTOCOL 1.0 SETTINGS
# ============================================================

BAUDRATE = 1_000_000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
COMM_SUCCESS = 0

MIN_SAFE_SPEED = 1
MAX_SAFE_SPEED = 1023
DEFAULT_SPEED = 25
DEFAULT_FRAMES = 25
DEFAULT_FRAME_DELAY = 0.045

RAW_PER_DEG = 1023.0 / 300.0


# ============================================================
# MOTOR MAP
# ============================================================

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
ALL_MOTOR_IDS = sorted(MOTOR_TO_JOINT.keys())


# ============================================================
# MODE 1: CLEAN AVERAGE / CODE-PERFECT VALUES
# ============================================================
# These are the simple shared numbers.
# No MR hip correction, no per-leg offset correction.

AVERAGE_HIP = 520
AVERAGE_FEMUR = 507
AVERAGE_TIBIA = 363


# ============================================================
# MODE 2: CALIBRATED PHYSICAL FLAT VALUES
# ============================================================
# These are the new physical lay-flat calibration values after the MR/RR
# inversion was fixed/reverted. Hips are still clean references.

CALIBRATED_NORMAL_HIP = 520
CALIBRATED_MR_HIP = 575

CALIBRATED_FEMUR = {
    "FL": 456,
    "ML": 529,
    "RL": 529,
    "FR": 513,
    "MR": 505,
    "RR": 507,
}

CALIBRATED_TIBIA = {
    "FL": 364,
    "ML": 364,
    "RL": 357,
    "FR": 364,
    "MR": 367,
    "RR": 361,
}


def clamp_raw(value: int) -> int:
    return int(max(0, min(1023, int(value))))


def raw_to_deg_from_520(raw: int) -> float:
    return (int(raw) - 520) / RAW_PER_DEG


def build_pose_average_perfect() -> Dict[int, int]:
    """
    Clean code-perfect average pose:
    all hips = 520, all femurs = 507, all tibias = 363.
    """
    pose: Dict[int, int] = {}
    for leg in LEG_ORDER:
        pose[LEG_JOINT_IDS[leg]["hip"]] = AVERAGE_HIP
        pose[LEG_JOINT_IDS[leg]["femur"]] = AVERAGE_FEMUR
        pose[LEG_JOINT_IDS[leg]["tibia"]] = AVERAGE_TIBIA
    return {mid: clamp_raw(raw) for mid, raw in pose.items()}


def build_pose_calibrated_physical() -> Dict[int, int]:
    """
    Physically calibrated flat pose:
    normal hips = 520, MR hip = 575, per-leg femur/tibia offsets applied.
    """
    pose: Dict[int, int] = {}

    for leg in LEG_ORDER:
        pose[LEG_JOINT_IDS[leg]["hip"]] = CALIBRATED_NORMAL_HIP
        pose[LEG_JOINT_IDS[leg]["femur"]] = CALIBRATED_FEMUR[leg]
        pose[LEG_JOINT_IDS[leg]["tibia"]] = CALIBRATED_TIBIA[leg]

    pose[LEG_JOINT_IDS["MR"]["hip"]] = CALIBRATED_MR_HIP
    return {mid: clamp_raw(raw) for mid, raw in pose.items()}


def pose_by_mode(mode: str) -> Dict[int, int]:
    mode = mode.lower().strip()
    if mode == "average_perfect":
        return build_pose_average_perfect()
    if mode == "calibrated_physical":
        return build_pose_calibrated_physical()
    raise ValueError(f"Unknown pose mode: {mode}")


def print_pose_table(name: str, pose: Dict[int, int]):
    print()
    print("===================================================")
    print(f" POSE: {name}")
    print("===================================================")
    print(f"{'Leg':<3} {'Joint':<5} {'ID':>2} {'Raw':>5} {'Deg from 520':>13}")
    print("-" * 38)

    for leg in LEG_ORDER:
        for part in ["hip", "femur", "tibia"]:
            motor_id = LEG_JOINT_IDS[leg][part]
            raw = int(pose[motor_id])
            print(f"{leg:<3} {part:<5} {motor_id:>2} {raw:>5} {raw_to_deg_from_520(raw):>+13.2f}")


def print_pose_dictionary(name: str, pose: Dict[int, int]):
    dict_name = name.upper() + "_POSE"
    print()
    print("===================================================")
    print(f" COPY-PASTE DICTIONARY: {dict_name}")
    print("===================================================")
    print(f"{dict_name} = {{")
    for leg in LEG_ORDER:
        for part in ["hip", "femur", "tibia"]:
            motor_id = LEG_JOINT_IDS[leg][part]
            raw = int(pose[motor_id])
            print(f"    {motor_id}: {raw:<4},  # {leg}_{part}")
    print("}")


# ============================================================
# SERIAL PORT SELECTION
# ============================================================

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
            index = int(choice) - 1
            if 0 <= index < len(ports):
                selected = ports[index][0]
                print(f"Selected: {selected}")
                return selected
            print("Invalid port number.")
            continue

        print(f"Selected manual port: {choice}")
        return choice


# ============================================================
# DYNAMIXEL BUS
# ============================================================

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

    def write1(self, motor_id: int, address: int, value: int) -> bool:
        try:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, int(motor_id), int(address), int(value)
            )
        except Exception as e:
            print(f"[ID {motor_id}] WRITE1 exception: {e}")
            return False

        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM error: {self.packet_handler.getTxRxResult(result)}")
            return False
        if error != 0:
            print(f"[ID {motor_id}] Packet error: {self.packet_handler.getRxPacketError(error)}")
            return False
        return True

    def write2(self, motor_id: int, address: int, value: int) -> bool:
        value = clamp_raw(value)
        try:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, int(motor_id), int(address), int(value)
            )
        except Exception as e:
            print(f"[ID {motor_id}] WRITE2 exception: {e}")
            return False

        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM error: {self.packet_handler.getTxRxResult(result)}")
            return False
        if error != 0:
            print(f"[ID {motor_id}] Packet error: {self.packet_handler.getRxPacketError(error)}")
            return False
        return True

    def read2(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, int(motor_id), int(address)
            )
        except Exception:
            return None

        if result != COMM_SUCCESS or error != 0:
            return None
        return int(value)

    def enable_torque_all(self):
        for motor_id in ALL_MOTOR_IDS:
            self.write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            time.sleep(0.004)

    def disable_torque_all(self):
        for motor_id in ALL_MOTOR_IDS:
            self.write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            time.sleep(0.004)

    def set_torque_limit_all(self, torque_limit: int = 1023):
        for motor_id in ALL_MOTOR_IDS:
            self.write2(motor_id, ADDR_TORQUE_LIMIT, torque_limit)
            time.sleep(0.004)

    def sync_set_speed(self, speed: int, motor_ids: Optional[List[int]] = None) -> bool:
        ids = list(motor_ids or ALL_MOTOR_IDS)
        speed = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(speed))))

        try:
            gsw = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_MOVING_SPEED, 2)
            for motor_id in ids:
                data = [speed & 0xFF, (speed >> 8) & 0xFF]
                gsw.addParam(int(motor_id), data)
            result = gsw.txPacket()
            gsw.clearParam()
            if result == COMM_SUCCESS:
                return True
        except Exception:
            pass

        for motor_id in ids:
            self.write2(motor_id, ADDR_MOVING_SPEED, speed)
        return True

    def sync_write_positions(self, targets: Dict[int, int]) -> bool:
        try:
            gsw = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, 2)
            for motor_id, raw in targets.items():
                raw = clamp_raw(raw)
                data = [raw & 0xFF, (raw >> 8) & 0xFF]
                gsw.addParam(int(motor_id), data)
            result = gsw.txPacket()
            gsw.clearParam()
            if result == COMM_SUCCESS:
                return True
        except Exception:
            pass

        ok = True
        for motor_id, raw in targets.items():
            ok = self.write2(motor_id, ADDR_GOAL_POSITION, raw) and ok
        return ok

    def read_current_positions(self, fallback_pose: Dict[int, int]) -> Dict[int, int]:
        current = {}
        for motor_id in ALL_MOTOR_IDS:
            pos = self.read2(motor_id, ADDR_PRESENT_POSITION)
            current[motor_id] = int(pos) if pos is not None else int(fallback_pose[motor_id])
        return current


# ============================================================
# MOVEMENT
# ============================================================

def interpolate(start: Dict[int, int], end: Dict[int, int], frames: int) -> List[Dict[int, int]]:
    frames = max(1, int(frames))
    output = []
    for i in range(1, frames + 1):
        ratio = i / frames
        frame = {}
        for motor_id in ALL_MOTOR_IDS:
            a = int(start.get(motor_id, end[motor_id]))
            b = int(end[motor_id])
            frame[motor_id] = clamp_raw(round(a + (b - a) * ratio))
        output.append(frame)
    return output


def print_readback(bus: DynamixelBus, pose: Dict[int, int]):
    print()
    print("===================================================")
    print(" FINAL READBACK")
    print("===================================================")
    print(f"{'ID':>2} {'Joint':<10} {'Target':>6} {'Read':>6} {'Error':>7}")
    print("-" * 40)

    for motor_id in ALL_MOTOR_IDS:
        target = int(pose[motor_id])
        actual = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        if actual is None:
            actual_s = "----"
            error_s = "----"
        else:
            actual_s = str(actual)
            error_s = f"{actual - target:+d}"
        print(f"{motor_id:>2} {MOTOR_TO_JOINT[motor_id]:<10} {target:>6} {actual_s:>6} {error_s:>7}")


def move_to_pose(bus: DynamixelBus, name: str, pose: Dict[int, int], speed: int, frames: int, delay: float):
    print_pose_table(name, pose)
    print_pose_dictionary(name, pose)

    print()
    print(f"Moving to pose: {name}")
    print(f"speed={speed}, frames={frames}, delay={delay:.3f}s")

    current = bus.read_current_positions(pose)

    bus.sync_set_speed(speed)
    for frame in interpolate(current, pose, frames):
        bus.sync_write_positions(frame)
        time.sleep(delay)

    bus.sync_set_speed(speed)
    bus.sync_write_positions(pose)
    time.sleep(0.50)
    bus.sync_write_positions(pose)
    time.sleep(0.50)

    print_readback(bus, pose)


def choose_mode_interactive() -> str:
    print()
    print("===================================================")
    print(" SELECT FLAT POSE MODE")
    print("===================================================")
    print("1) average_perfect      - all hips 520, all femurs 507, all tibias 363")
    print("2) calibrated_physical  - MR hip 575 + per-leg femur/tibia offset calibration")
    print("3) compare              - average_perfect first, then calibrated_physical")
    print()

    while True:
        choice = input("Mode [calibrated_physical]: ").strip().lower()
        if choice == "":
            return "calibrated_physical"
        if choice in ["1", "average", "average_perfect", "perfect"]:
            return "average_perfect"
        if choice in ["2", "calibrated", "calibrated_physical", "physical"]:
            return "calibrated_physical"
        if choice in ["3", "compare"]:
            return "compare"
        print("Invalid mode. Choose 1, 2, or 3.")


def main():
    parser = argparse.ArgumentParser(description="Move hexapod to average-perfect or calibrated-physical flat pose.")
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM6 or /dev/ttyUSB0. If omitted, menu is shown.")
    parser.add_argument("--mode", default=None, choices=["average_perfect", "calibrated_physical", "compare"], help="Pose mode.")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="Dynamixel moving speed. Default: 25.")
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAMES, help="Smooth interpolation frames. Default: 25.")
    parser.add_argument("--delay", type=float, default=DEFAULT_FRAME_DELAY, help="Delay per frame. Default: 0.045s.")
    parser.add_argument("--hold", action="store_true", help="Keep script open after moving so torque stays on until Enter.")
    parser.add_argument("--disable-torque-on-exit", action="store_true", help="Disable torque before closing.")
    args = parser.parse_args()

    mode = args.mode or choose_mode_interactive()

    print()
    print("SAFETY:")
    print("- This script WILL move all 18 motors.")
    print("- Put the robot on a stand/support first.")
    print("- Keep power switch nearby.")
    print("- No YOLO/tracking. This is only flat-pose motor calibration.")

    port = args.port or choose_serial_port()
    bus = DynamixelBus(port)

    if not bus.open():
        sys.exit(1)

    try:
        speed = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, args.speed)))
        frames = max(1, int(args.frames))
        delay = max(0.0, float(args.delay))

        print()
        print("Enabling torque and setting speed...")
        bus.enable_torque_all()
        bus.set_torque_limit_all(1023)
        bus.sync_set_speed(speed)

        if mode == "compare":
            avg_pose = build_pose_average_perfect()
            move_to_pose(bus, "average_perfect", avg_pose, speed, frames, delay)

            print()
            input("Observe the average/code-perfect pose. Press Enter to move to calibrated_physical...")

            cal_pose = build_pose_calibrated_physical()
            move_to_pose(bus, "calibrated_physical", cal_pose, speed, frames, delay)
        else:
            pose = pose_by_mode(mode)
            move_to_pose(bus, mode, pose, speed, frames, delay)

        print()
        print("Done.")

        if args.hold:
            input("Press Enter to close the script...")

        if args.disable_torque_on_exit:
            print("Disabling torque...")
            bus.disable_torque_all()

    finally:
        bus.close()


if __name__ == "__main__":
    main()
