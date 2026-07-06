"""
hexapod_controller_client_rockfire_fullmode_v4_speedstep.py

Final-ish Rockfire PSX/USB Bridge client based on your controller mode log.

Confirmed full-mode mapping from your latest log:

D-PAD / left-side 4 direction buttons:
    hat 0 (0, 1)   = Up
    hat 0 (1, 0)   = Right
    hat 0 (0, -1)  = Down
    hat 0 (-1, 0)  = Left

Left joystick:
    axis 0 = left/right
    axis 1 = up/down

Right joystick:
    axis 2 = left/right
    axis 3 = up/down
    For robot turning, we use axis 2:
        axis 2 +1 = turn right
        axis 2 -1 = turn left

Face buttons:
    button 0 = Cross
    button 1 = Circle
    button 2 = Square
    button 3 = Triangle

Shoulder / trigger buttons:
    button 4 = L1
    button 5 = R1
    button 6 = L2
    button 7 = R2

Robot mapping:
    D-pad / left joystick:
        Up      = forward
        Down    = backward
        Left    = strafe left
        Right   = strafe right

    Right joystick:
        Left    = turn left
        Right   = turn right

    Cross:
        reset body-height level to 0

    Circle:
        ready pose at current body-height level

    Square:
        health check

    Triangle:
        startup setup + reset body-height level to 0

    L1:
        speed all 20

    R1:
        speed all 25

    L2 hold:
        lower body-height level toward -7

    R2 hold:
        raise body-height level toward +7

Use with Raspberry Pi server:
    python3 SControlX2_web_ui_v8_smooth_body_height.py

Install:
    pip install pygame requests

Run:
    python hexapod_controller_client_rockfire_fullmode_v4_speedstep.py
"""

import time
from dataclasses import dataclass
from typing import Optional

try:
    import pygame
except ImportError:
    print("Missing pygame. Install with: pip install pygame")
    raise SystemExit(1)

try:
    import requests
except ImportError:
    print("Missing requests. Install with: pip install requests")
    raise SystemExit(1)


# ============================================================
# CONFIG
# ============================================================

DEFAULT_BASE_URL = "http://raspberrypi.local:8000"

DEADZONE = 0.45
LOOP_DELAY = 0.035
STATE_POLL_INTERVAL = 0.20
BODY_STEP_INTERVAL = 0.55

BODY_MIN = -7
BODY_MAX = +7

SPEED_MIN = 1
SPEED_MAX = 1023
CURRENT_SPEED = 25

# Axes from full-mode log
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
RIGHT_X_AXIS = 2
RIGHT_Y_AXIS = 3

# Buttons from full-mode log
BTN_CROSS = 0
BTN_CIRCLE = 1
BTN_SQUARE = 2
BTN_TRIANGLE = 3
BTN_L1 = 4
BTN_R1 = 5
BTN_L2 = 6
BTN_R2 = 7

# Optional/unconfirmed buttons
BTN_SELECT = 8
BTN_START = 9
BTN_ANALOG = 10
BTN_EXTRA = 11


# ============================================================
# API
# ============================================================

@dataclass
class Api:
    base_url: str

    def url(self, path):
        return self.base_url.rstrip("/") + path

    def get_state(self):
        try:
            r = requests.get(self.url("/api/state"), timeout=1.5)
            if r.status_code >= 300:
                print(f"[HTTP] GET /api/state {r.status_code}: {r.text[:120]}")
                return None
            return r.json()
        except Exception as e:
            print(f"[HTTP] GET /api/state failed: {e}")
            return None

    def post(self, path, payload, timeout=2.0):
        try:
            r = requests.post(self.url(path), json=payload, timeout=timeout)
            if r.status_code >= 300:
                print(f"[HTTP] POST {path} {r.status_code}: {r.text[:120]}")
                return False
            return True
        except Exception as e:
            print(f"[HTTP] POST {path} failed: {e}")
            return False

    def command(self, cmd):
        ok = self.post("/api/command", {"command": cmd})
        if ok:
            print(f"[SEND] COMMAND: {cmd}")
        return ok

    def start_move(self, direction):
        ok = self.post("/api/move/start", {"direction": direction})
        if ok:
            print(f"[SEND] START {direction}")
        return ok

    def stop(self):
        ok = self.post("/api/move/stop", {})
        if ok:
            print("[SEND] STOP / RETURN")
        return ok

    def body_delta(self, delta):
        ok = self.post("/api/action/bodylevel", {"mode": "delta", "level": 0, "delta": int(delta)})
        if ok:
            print(f"[SEND] BODY LEVEL DELTA {delta:+d}")
        return ok

    def body_reset(self):
        ok = self.post("/api/action/bodylevel", {"mode": "reset", "level": 0, "delta": 0})
        if ok:
            print("[SEND] BODY LEVEL RESET 0")
        return ok


# ============================================================
# INPUT HELPERS
# ============================================================

def btn(joy, index):
    if index < 0 or index >= joy.get_numbuttons():
        return False
    try:
        return bool(joy.get_button(index))
    except Exception:
        return False


def axis_raw(joy, index):
    if index < 0 or index >= joy.get_numaxes():
        return 0.0
    try:
        return float(joy.get_axis(index))
    except Exception:
        return 0.0


def axis(joy, index):
    value = axis_raw(joy, index)
    if abs(value) < DEADZONE:
        return 0.0
    return value


def hat(joy):
    if joy.get_numhats() <= 0:
        return (0, 0)
    try:
        return joy.get_hat(0)
    except Exception:
        return (0, 0)


def motion_state(st):
    if not st:
        return "unknown"
    return str(st.get("motion", "unknown") or "unknown").lower()


def body_level(st, fallback=0):
    try:
        return int((st.get("body_height") or {}).get("level", fallback))
    except Exception:
        return fallback


def clamp_speed(speed):
    return max(SPEED_MIN, min(SPEED_MAX, int(speed)))


def send_speed(api, speed):
    speed = clamp_speed(speed)
    api.command(f"speed all {speed}")
    return speed


def get_dpad_direction(joy) -> Optional[str]:
    x, y = hat(joy)
    if y > 0:
        return "forward"
    if y < 0:
        return "backward"
    if x < 0:
        return "left"
    if x > 0:
        return "right"
    return None


def get_left_stick_direction(joy) -> Optional[str]:
    x = axis(joy, LEFT_X_AXIS)
    y = axis(joy, LEFT_Y_AXIS)

    if x == 0.0 and y == 0.0:
        return None

    if abs(y) >= abs(x):
        return "forward" if y < 0 else "backward"

    return "left" if x < 0 else "right"


def get_right_stick_turn(joy) -> Optional[str]:
    x = axis(joy, RIGHT_X_AXIS)

    if x == 0.0:
        return None

    # Based on your latest log:
    # axis 2 +1 then -1 were observed. If physical direction feels inverted,
    # swap these two returns.
    return "turn_right" if x > 0 else "turn_left"


def desired_direction(joy) -> Optional[str]:
    # Priority:
    # 1. D-pad for clean digital movement
    # 2. Right stick horizontal for turning
    # 3. Left joystick for movement
    dpad_dir = get_dpad_direction(joy)
    if dpad_dir:
        return dpad_dir

    turn_dir = get_right_stick_turn(joy)
    if turn_dir:
        return turn_dir

    return get_left_stick_direction(joy)


def wait_for_controller():
    pygame.joystick.quit()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count <= 0:
        print("No controller detected.")
        raise SystemExit(1)

    print()
    print("Detected controllers:")
    for i in range(count):
        j = pygame.joystick.Joystick(i)
        j.init()
        print(f"  {i}) {j.get_name()} | buttons={j.get_numbuttons()} axes={j.get_numaxes()} hats={j.get_numhats()}")

    choice = input("Select controller index [0]: ").strip() or "0"
    joy = pygame.joystick.Joystick(int(choice))
    joy.init()
    return joy


def print_controls():
    print()
    print("===================================================")
    print(" ROCKFIRE FULL-MODE CONTROLS V5 SMOOTH HEIGHT")
    print("===================================================")
    print("D-PAD")
    print("  Up       = Forward")
    print("  Down     = Backward")
    print("  Left     = Strafe Left")
    print("  Right    = Strafe Right")
    print()
    print("LEFT JOYSTICK")
    print("  Axis 1 -1 = Forward")
    print("  Axis 1 +1 = Backward")
    print("  Axis 0 -1 = Strafe Left")
    print("  Axis 0 +1 = Strafe Right")
    print()
    print("RIGHT JOYSTICK")
    print("  Axis 2 -1 = Turn Left")
    print("  Axis 2 +1 = Turn Right")
    print()
    print("FACE BUTTONS")
    print("  Button 0 / Cross     = Reset body-height level to 0")
    print("  Button 1 / Circle    = Ready at current body-height level")
    print("  Button 2 / Square    = Health")
    print("  Button 3 / Triangle  = Startup setup + reset body-height level")
    print()
    print("SHOULDER / TRIGGER")
    print("  Button 4 / L1        = Speed -1 per click")
    print("  Button 5 / R1        = Speed +1 per click")
    print("  Button 6 / L2 hold   = Smooth lower body toward -7")
    print("  Button 7 / R2 hold   = Smooth raise body toward +7")
    print()
    print("BEHAVIOR")
    print("  Hold movement        = Move")
    print("  Release movement     = Stop / return")
    print("  Robot busy           = New directions ignored")
    print("  Idle + neutral       = Accept next command")
    print("===================================================")
    print()


# ============================================================
# MAIN
# ============================================================

def main():
    print()
    print("===================================================")
    print(" HEXAPOD ROCKFIRE FULL-MODE CLIENT V5 SMOOTH HEIGHT")
    print("===================================================")

    url = input(f"Raspberry Pi web URL [{DEFAULT_BASE_URL}]: ").strip() or DEFAULT_BASE_URL
    api = Api(url)

    st = api.get_state()
    if st is None:
        print("Cannot reach server. Run on Pi:")
        print("  python3 SControlX2_web_ui_v8_smooth_body_height.py")
        raise SystemExit(1)

    pygame.init()
    pygame.joystick.init()

    joy = wait_for_controller()
    print_controls()

    active = None
    neutral_gate = False
    server_motion = motion_state(st)
    current_body = body_level(st, 0)
    current_speed = CURRENT_SPEED

    prev = {}
    last_poll = 0.0
    last_body_step = 0.0
    last_debug = 0.0

    try:
        while True:
            now = time.time()
            pygame.event.pump()

            if now - last_poll >= STATE_POLL_INTERVAL:
                latest = api.get_state()
                if latest is not None:
                    server_motion = motion_state(latest)
                    current_body = body_level(latest, current_body)
                last_poll = now

            idle = server_motion == "idle"
            desired = desired_direction(joy)

            # Hold/release movement gate
            if active is not None and desired is None:
                api.stop()
                active = None
                neutral_gate = True
                time.sleep(0.05)

            elif active is not None and desired != active:
                api.stop()
                print(f"[GATE] Direction changed {active} -> {desired}; stopping only.")
                active = None
                neutral_gate = True
                time.sleep(0.05)

            if neutral_gate and idle and desired is None:
                neutral_gate = False
                print("[GATE] idle + neutral, ready.")

            if active is None and desired is not None:
                if idle and not neutral_gate:
                    if api.start_move(desired):
                        active = desired
                else:
                    neutral_gate = True

            # Body height hold: L2 lower, R2 raise
            lower = btn(joy, BTN_L2)
            raise_ = btn(joy, BTN_R2)

            if idle and active is None and now - last_body_step >= BODY_STEP_INTERVAL:
                if lower and not raise_:
                    if current_body > BODY_MIN:
                        api.body_delta(-1)
                        current_body -= 1
                    else:
                        print("[BODY] already lowest -7")
                    last_body_step = now

                elif raise_ and not lower:
                    if current_body < BODY_MAX:
                        api.body_delta(+1)
                        current_body += 1
                    else:
                        print("[BODY] already highest +7")
                    last_body_step = now

                elif lower and raise_:
                    print("[BODY] L2 and R2 both active; ignoring.")
                    last_body_step = now

            # Button edge actions
            actions = {
                BTN_CROSS: "body_reset",
                BTN_CIRCLE: "ready",
                BTN_SQUARE: "health",
                BTN_TRIANGLE: "startup",
                BTN_L1: "speed_down",
                BTN_R1: "speed_up",
                BTN_SELECT: "stop",
                BTN_START: "body_reset",
            }

            for b, action in actions.items():
                pressed = btn(joy, b)
                old = prev.get(b, False)

                if pressed and not old:
                    if action == "body_reset":
                        if idle:
                            api.body_reset()
                            current_body = 0
                        else:
                            print(f"[GATE] body reset ignored, not idle ({server_motion})")

                    elif action == "ready":
                        api.command("r")

                    elif action == "health":
                        api.command("health")

                    elif action == "startup":
                        for cmd, delay in [
                            ("bodylevel reset", 0.12),
                            ("r", 0.15),
                            ("health", 0.15),
                            ("sidestrafe good", 0.08),
                            ("movestats off", 0.08),
                            ("sideflow on", 0.08),
                            ("speed all 25", 0.0),
                        ]:
                            api.command(cmd)
                            if delay:
                                time.sleep(delay)
                        current_body = 0

                    elif action == "speed_down":
                        current_speed = send_speed(api, current_speed - 1)
                        print(f"[SPEED] speed all {current_speed}")

                    elif action == "speed_up":
                        current_speed = send_speed(api, current_speed + 1)
                        print(f"[SPEED] speed all {current_speed}")

                    elif action == "stop":
                        api.stop()
                        active = None
                        neutral_gate = True

                prev[b] = pressed

            if now - last_debug >= 0.5:
                last_debug = now
                button_states = " ".join([f"b{i}={int(btn(joy, i))}" for i in range(min(12, joy.get_numbuttons()))])
                hx, hy = hat(joy)
                print(
                    f"[DEBUG] server={server_motion:<10} idle={idle} body={current_body:+d} speed={current_speed} "
                    f"desired={str(desired):<10} active={str(active):<10} gate={neutral_gate} "
                    f"hat=({hx},{hy}) "
                    f"lx={axis(joy, LEFT_X_AXIS):+.2f} ly={axis(joy, LEFT_Y_AXIS):+.2f} "
                    f"rx={axis(joy, RIGHT_X_AXIS):+.2f} ry={axis(joy, RIGHT_Y_AXIS):+.2f} "
                    f"{button_states}"
                )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C. Sending STOP.")
        api.stop()

    finally:
        pygame.joystick.quit()
        pygame.quit()
        print("Controller closed.")


if __name__ == "__main__":
    main()
