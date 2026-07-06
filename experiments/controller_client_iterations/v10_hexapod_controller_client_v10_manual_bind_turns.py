
"""
hexapod_controller_client_v10_manual_bind_turns.py

Laptop-side controller client for SControlX2 body-height web server.

Fixes:
- Manual bind for L2 and R2 body-height controls.
- Manual bind for TURN LEFT and TURN RIGHT, because Rockfire adapters may not use
  the guessed right-stick axis.
- Explains calibration prompts clearly.
- Movement/strafing still uses D-pad / left joystick.
- Turning uses the manually bound inputs.

Use with Raspberry Pi server:
    python3 SControlX2_web_ui_v7_body_height_deeper.py

Install:
    pip install pygame requests

Run:
    python hexapod_controller_client_v10_manual_bind_turns.py
"""

import time
from dataclasses import dataclass

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


DEFAULT_BASE_URL = "http://raspberrypi.local:8000"

DEADZONE = 0.40
AXIS_CHANGE_THRESHOLD = 0.28
BODY_STEP_INTERVAL = 0.32
STATE_POLL_INTERVAL = 0.20
LOOP_DELAY = 0.035

BODY_MIN = -7
BODY_MAX = +7

BUTTON_CROSS_A = 0
BUTTON_CIRCLE_B = 1
BUTTON_SQUARE_X = 2
BUTTON_TRIANGLE_Y = 3
BUTTON_L1_LB = 4
BUTTON_R1_RB = 5
BUTTON_SELECT_BACK = 6
BUTTON_START = 7

LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1


@dataclass
class Binding:
    kind: str = "none"
    index: int = -1
    neutral: float = 0.0
    sign: int = 1

    def describe(self):
        if self.kind == "button":
            return f"button {self.index}"
        if self.kind == "axis":
            return f"axis {self.index}, neutral={self.neutral:+.3f}, sign={'+' if self.sign > 0 else '-'}"
        return "none"


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

    def command(self, c):
        ok = self.post("/api/command", {"command": c})
        if ok:
            print(f"[SEND] COMMAND: {c}")
        return ok

    def start_move(self, d):
        ok = self.post("/api/move/start", {"direction": d})
        if ok:
            print(f"[SEND] START {d}")
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


def button(joy, i):
    if i < 0 or i >= joy.get_numbuttons():
        return False
    try:
        return bool(joy.get_button(i))
    except Exception:
        return False


def axis_raw(joy, i):
    if i is None or i < 0 or i >= joy.get_numaxes():
        return 0.0
    try:
        return float(joy.get_axis(i))
    except Exception:
        return 0.0


def axis_value(joy, i):
    v = axis_raw(joy, i)
    return 0.0 if abs(v) < DEADZONE else v


def snapshot_buttons_axes(joy):
    pygame.event.pump()
    buttons = [1 if button(joy, i) else 0 for i in range(joy.get_numbuttons())]
    axes = [axis_raw(joy, i) for i in range(joy.get_numaxes())]
    return buttons, axes


def bind_by_press(joy, name):
    print()
    print("===================================================")
    print(f" BIND {name}")
    print("===================================================")
    print("Step 1: Release ALL controller buttons/sticks/triggers.")
    input("Press Enter when everything is neutral...")

    neutral_buttons, neutral_axes = snapshot_buttons_axes(joy)

    print()
    print(f"Step 2: Press/HOLD or move {name}.")
    print("Example: for TURN LEFT, push the right joystick LEFT and hold it.")
    input(f"While holding {name}, press Enter...")

    active_buttons, active_axes = snapshot_buttons_axes(joy)

    for i in range(min(len(neutral_buttons), len(active_buttons))):
        if neutral_buttons[i] == 0 and active_buttons[i] == 1:
            b = Binding("button", i)
            print(f"{name} bound to {b.describe()}")
            return b

    best_i = -1
    best_delta = 0.0
    for i in range(min(len(neutral_axes), len(active_axes))):
        delta = active_axes[i] - neutral_axes[i]
        if abs(delta) > abs(best_delta):
            best_delta = delta
            best_i = i

    if best_i >= 0 and abs(best_delta) >= AXIS_CHANGE_THRESHOLD:
        b = Binding("axis", best_i, neutral_axes[best_i], 1 if best_delta > 0 else -1)
        print(f"{name} bound to {b.describe()}")
        return b

    print(f"Could not detect {name}.")
    return Binding("none")


def binding_pressed(joy, bind: Binding):
    if bind.kind == "button":
        return button(joy, bind.index)

    if bind.kind == "axis":
        v = axis_raw(joy, bind.index)
        delta = v - bind.neutral
        if bind.sign > 0:
            return delta >= AXIS_CHANGE_THRESHOLD
        return delta <= -AXIS_CHANGE_THRESHOLD

    return False


def get_dpad_direction(joy):
    if joy.get_numhats() <= 0:
        return None
    x, y = joy.get_hat(0)
    if y > 0:
        return "forward"
    if y < 0:
        return "backward"
    if x < 0:
        return "left"
    if x > 0:
        return "right"
    return None


def get_left_stick_direction(joy):
    lx = axis_value(joy, LEFT_X_AXIS)
    ly = axis_value(joy, LEFT_Y_AXIS)
    if lx == 0.0 and ly == 0.0:
        return None
    if abs(ly) >= abs(lx):
        return "forward" if ly < 0 else "backward"
    return "left" if lx < 0 else "right"


def desired_direction(joy, turn_left_bind, turn_right_bind):
    dpad = get_dpad_direction(joy)
    if dpad:
        return dpad

    tl = binding_pressed(joy, turn_left_bind)
    tr = binding_pressed(joy, turn_right_bind)
    if tl and not tr:
        return "turn_left"
    if tr and not tl:
        return "turn_right"

    return get_left_stick_direction(joy)


def motion_state(st):
    if not st:
        return "unknown"
    return str(st.get("motion", "unknown") or "unknown").lower()


def body_level(st, fallback=0):
    try:
        return int((st.get("body_height") or {}).get("level", fallback))
    except Exception:
        return fallback


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

    idx = input("Select controller index [0]: ").strip() or "0"
    joy = pygame.joystick.Joystick(int(idx))
    joy.init()
    return joy


def print_controls(joy, l2, r2, turn_left, turn_right):
    print()
    print("===================================================")
    print(" CONTROLS - V10 MANUAL BIND TURNS")
    print("===================================================")
    print("D-PAD / LEFT JOYSTICK")
    print("  Up/Down/Left/Right = Forward/Backward/Strafe")
    print()
    print("TURNING")
    print("  Turn Left  = your manually bound Turn Left input")
    print("  Turn Right = your manually bound Turn Right input")
    print()
    print("FACE BUTTONS")
    print("  Cross / A     = Reset body-height level to 0")
    print("  Circle / B    = Ready at current body-height level")
    print("  Square / X    = Health")
    print("  Triangle / Y  = Startup setup + reset body-height level")
    print()
    print("BODY HEIGHT")
    print("  L2 hold       = Lower body one step every 0.32s until -7")
    print("  R2 hold       = Raise body one step every 0.32s until +7")
    print("  Release       = Stay at current level")
    print()
    print("SHOULDERS / MIDDLE")
    print("  L1            = speed all 20")
    print("  R1            = speed all 25")
    print("  Select/Back   = STOP / Return")
    print("  Start         = Reset body-height level to 0")
    print("===================================================")
    print(f"Controller: {joy.get_name()}")
    print(f"Buttons={joy.get_numbuttons()} Axes={joy.get_numaxes()} Hats={joy.get_numhats()}")
    print(f"L2 binding         = {l2.describe()}")
    print(f"R2 binding         = {r2.describe()}")
    print(f"Turn Left binding  = {turn_left.describe()}")
    print(f"Turn Right binding = {turn_right.describe()}")
    print("===================================================")


def main():
    print()
    print("===================================================")
    print(" HEXAPOD CONTROLLER CLIENT V10 - MANUAL TURN BIND")
    print("===================================================")
    print("About the Enter prompts:")
    print("They are calibration snapshots only. They do NOT send robot commands.")
    print("Each bind has:")
    print("  1) neutral snapshot: release everything, press Enter")
    print("  2) active snapshot : hold/move target input, press Enter")
    print("===================================================")

    url = input(f"Raspberry Pi web URL [{DEFAULT_BASE_URL}]: ").strip() or DEFAULT_BASE_URL
    api = Api(url)

    st = api.get_state()
    if st is None:
        print("Cannot reach server. Run on Pi:")
        print("  python3 SControlX2_web_ui_v7_body_height_deeper.py")
        raise SystemExit(1)

    pygame.init()
    pygame.joystick.init()

    joy = wait_for_controller()

    l2 = bind_by_press(joy, "L2 / LT")
    print("Release L2 now.")
    time.sleep(0.5)
    r2 = bind_by_press(joy, "R2 / RT")
    print("Release R2 now.")
    time.sleep(0.5)

    turn_left = bind_by_press(joy, "TURN LEFT")
    print("Release turn-left input now.")
    time.sleep(0.5)
    turn_right = bind_by_press(joy, "TURN RIGHT")
    print("Release turn-right input now.")
    time.sleep(0.5)

    print_controls(joy, l2, r2, turn_left, turn_right)

    active = None
    neutral_gate = False
    server_motion = motion_state(st)
    current_body = body_level(st, 0)
    prev_buttons = {}
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
            desired = desired_direction(joy, turn_left, turn_right)

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

            l2_now = binding_pressed(joy, l2)
            r2_now = binding_pressed(joy, r2)

            if idle and active is None and now - last_body_step >= BODY_STEP_INTERVAL:
                if l2_now and not r2_now:
                    if current_body > BODY_MIN:
                        api.body_delta(-1)
                        current_body -= 1
                    else:
                        print("[BODY] already lowest -7")
                    last_body_step = now

                elif r2_now and not l2_now:
                    if current_body < BODY_MAX:
                        api.body_delta(+1)
                        current_body += 1
                    else:
                        print("[BODY] already highest +7")
                    last_body_step = now

                elif l2_now and r2_now:
                    print("[BODY] L2 and R2 both active; ignoring.")
                    last_body_step = now

            actions = {
                BUTTON_CROSS_A: "body_reset",
                BUTTON_CIRCLE_B: "ready",
                BUTTON_SQUARE_X: "health",
                BUTTON_TRIANGLE_Y: "startup",
                BUTTON_L1_LB: "speed20",
                BUTTON_R1_RB: "speed25",
                BUTTON_SELECT_BACK: "stop",
                BUTTON_START: "body_reset",
            }

            for b, action in actions.items():
                pressed = button(joy, b)
                old = prev_buttons.get(b, False)

                if pressed and not old:
                    if action == "stop":
                        api.stop()
                        active = None
                        neutral_gate = True
                    elif action == "body_reset":
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
                    elif action == "speed20":
                        api.command("speed all 20")
                    elif action == "speed25":
                        api.command("speed all 25")

                prev_buttons[b] = pressed

            if now - last_debug >= 0.5:
                last_debug = now
                dpad = joy.get_hat(0) if joy.get_numhats() else None
                tl_now = binding_pressed(joy, turn_left)
                tr_now = binding_pressed(joy, turn_right)
                print(
                    f"[DEBUG] server={server_motion:<10} idle={idle} body={current_body:+d} "
                    f"desired={str(desired):<10} active={str(active):<10} "
                    f"L2={l2_now} R2={r2_now} TL={tl_now} TR={tr_now} "
                    f"Dpad={dpad} LX={axis_value(joy, LEFT_X_AXIS):+.2f} "
                    f"LY={axis_value(joy, LEFT_Y_AXIS):+.2f}"
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
