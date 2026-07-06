"""
controller_select_and_print.py

Purpose:
- List all detected joystick/controller devices.
- Let you select which one to test.
- Print ONLY actual controller input events:
  button down/up, D-pad/hat changes, and joystick axis changes.
- Helpful for checking old USB adapters such as Rockfire USB controller adapters.

Install:
    pip install pygame

Run:
    python controller_select_and_print.py

Exit:
    Press Ctrl+C in terminal.
"""

import time
import csv
from datetime import datetime
from pathlib import Path

try:
    import pygame
except ImportError:
    print("Missing pygame.")
    print("Install it with:")
    print("    pip install pygame")
    raise SystemExit(1)


LOG_FILE = Path("controller_selected_input_log.csv")

# Increase this if analog sticks are noisy.
AXIS_DEADZONE = 0.15

# Only reprint axis if value changes by this much.
AXIS_CHANGE_PRINT = 0.08

# Some adapters constantly jitter analog axes.
# This prevents spam.
AXIS_MIN_PRINT_INTERVAL = 0.10


def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def wait_for_enter(message="Press Enter to continue..."):
    try:
        input(message)
    except KeyboardInterrupt:
        raise


def init_pygame():
    pygame.init()
    pygame.joystick.init()


def refresh_joysticks():
    pygame.joystick.quit()
    pygame.joystick.init()

    devices = []
    count = pygame.joystick.get_count()

    for i in range(count):
        joy = pygame.joystick.Joystick(i)
        joy.init()
        devices.append(joy)

    return devices


def print_devices(devices):
    print()
    print("===================================================")
    print(" AVAILABLE CONTROLLER / JOYSTICK DEVICES")
    print("===================================================")

    if not devices:
        print("No controller/joystick detected by pygame.")
        print()
        print("Try these checks:")
        print("1. Replug the Rockfire USB adapter.")
        print("2. Try another USB port.")
        print("3. Open Windows Game Controllers:")
        print("      Win + R")
        print("      joy.cpl")
        print("4. Check if the controller appears there first.")
        print("5. If using old PS2-style controller adapter, press Analog/Mode button if available.")
        print("6. Restart this script after reconnecting.")
        return

    for idx, joy in enumerate(devices):
        print(f"{idx}) {joy.get_name()}")
        print(f"   Instance ID : {joy.get_instance_id()}")
        print(f"   GUID        : {joy.get_guid()}")
        print(f"   Buttons     : {joy.get_numbuttons()}")
        print(f"   Axes        : {joy.get_numaxes()}")
        print(f"   Hats/D-pad  : {joy.get_numhats()}")
        print()


def choose_device():
    while True:
        devices = refresh_joysticks()
        print_devices(devices)

        print()
        print("Options:")
        print("  r       = rescan")
        print("  q       = quit")
        print("  number  = select controller index, e.g. 0")

        choice = input("Select option: ").strip().lower()

        if choice == "q":
            raise SystemExit(0)

        if choice in ["r", "rescan", "scan"]:
            continue

        if choice.isdigit():
            index = int(choice)
            if 0 <= index < len(devices):
                return index
            print("Invalid controller index.")
            time.sleep(0.5)
            continue

        print("Invalid option.")
        time.sleep(0.5)


def apply_deadzone(value):
    value = float(value)
    if abs(value) < AXIS_DEADZONE:
        return 0.0
    return value


def print_current_raw_state(joy):
    print()
    print("===================================================")
    print(" CURRENT RAW STATE SNAPSHOT")
    print("===================================================")

    print("Buttons:")
    if joy.get_numbuttons() == 0:
        print("  No buttons reported.")
    for i in range(joy.get_numbuttons()):
        print(f"  button {i:<2}: {joy.get_button(i)}")

    print("Axes:")
    if joy.get_numaxes() == 0:
        print("  No axes reported.")
    for i in range(joy.get_numaxes()):
        print(f"  axis {i:<2}: {joy.get_axis(i):+.3f}")

    print("Hats / D-pad:")
    if joy.get_numhats() == 0:
        print("  No hats/D-pad reported.")
    for i in range(joy.get_numhats()):
        print(f"  hat {i:<2}: {joy.get_hat(i)}")

    print("===================================================")


def test_selected_controller(index):
    # Re-open selected joystick after menu selection.
    joy = pygame.joystick.Joystick(index)
    joy.init()

    print()
    print("===================================================")
    print(" TESTING SELECTED CONTROLLER")
    print("===================================================")
    print(f"Selected index : {index}")
    print(f"Name           : {joy.get_name()}")
    print(f"Buttons        : {joy.get_numbuttons()}")
    print(f"Axes           : {joy.get_numaxes()}")
    print(f"Hats/D-pad     : {joy.get_numhats()}")
    print("===================================================")
    print()
    print("Now press buttons, move sticks, or press D-pad.")
    print("Only real changes will be printed.")
    print("Press Ctrl+C to stop.")
    print()

    print_current_raw_state(joy)

    # Previous states for change detection
    prev_buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    prev_axes = [apply_deadzone(joy.get_axis(i)) for i in range(joy.get_numaxes())]
    prev_hats = [joy.get_hat(i) for i in range(joy.get_numhats())]
    last_axis_print_time = [0.0 for _ in range(joy.get_numaxes())]

    with LOG_FILE.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "type", "index", "value"])

        def log(event_type, control_index, value):
            writer.writerow([datetime.now().isoformat(timespec="milliseconds"), event_type, control_index, value])
            f.flush()

        try:
            while True:
                pygame.event.pump()

                # Buttons
                for i in range(joy.get_numbuttons()):
                    value = joy.get_button(i)
                    if value != prev_buttons[i]:
                        prev_buttons[i] = value
                        state = "DOWN" if value else "UP"
                        print(f"[{ts()}] BUTTON {state:<4} button={i}")
                        log(f"BUTTON_{state}", i, value)

                # Axes
                now = time.time()
                for i in range(joy.get_numaxes()):
                    value = apply_deadzone(joy.get_axis(i))
                    old = prev_axes[i]

                    if abs(value - old) >= AXIS_CHANGE_PRINT:
                        if now - last_axis_print_time[i] >= AXIS_MIN_PRINT_INTERVAL:
                            prev_axes[i] = value
                            last_axis_print_time[i] = now
                            print(f"[{ts()}] AXIS       axis={i}, value={value:+.3f}")
                            log("AXIS", i, f"{value:+.3f}")

                # Hats / D-pad
                for i in range(joy.get_numhats()):
                    value = joy.get_hat(i)
                    if value != prev_hats[i]:
                        prev_hats[i] = value
                        print(f"[{ts()}] HAT/D-PAD   hat={i}, value={value}")
                        log("HAT", i, value)

                time.sleep(0.01)

        except KeyboardInterrupt:
            print()
            print("Stopped testing selected controller.")

    print()
    print("===================================================")
    print(" TEST FINISHED")
    print("===================================================")
    print(f"Saved log: {LOG_FILE.resolve()}")


def main():
    init_pygame()

    print()
    print("===================================================")
    print(" ROCKFIRE / USB CONTROLLER DETECTION TEST")
    print("===================================================")
    print("This script lists available controller devices first.")
    print("Then you select one and it prints button/stick/D-pad inputs.")
    print("===================================================")

    try:
        index = choose_device()
        test_selected_controller(index)
    finally:
        pygame.joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    main()
