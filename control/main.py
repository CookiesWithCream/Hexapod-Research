
from __future__ import annotations

"""
Hexapod-only controller launcher.

Goal:
- One Python process owns the Dynamixel COM port.
- Keep the classic IKControl-style web UI for documentation/demo.
- Add runtime Serial/COM port listing and safe COM switching.
- Keep terminal/web command support for all hexapod IKControl features.
- Remove the vision page, model switch, camera routes, and vision terminal commands.

Run examples:
    python main.py --mode web --com COM6
    python main.py --mode terminal --com COM6
    python main.py                 # asks for mode and serial port
"""

import argparse
import atexit
import glob
import logging
import os
import threading
import time
import webbrowser
from html import escape as html_escape
from typing import Any, Optional

from flask import Flask, jsonify, redirect, request

import IKControl as ik

WEB_HOST = "0.0.0.0"  # Bind to all network interfaces so laptop can access Pi over LAN
WEB_PORT = 8000
LOCAL_BROWSER_HOST = "127.0.0.1"

# One re-entrant lock guards raw serial transactions.
SERIAL_LOCK = threading.RLock()


# -----------------------------------------------------------------------------
# 1) Thread-safe IK bus: same IKControl.DynamixelBus API, but serial operations
#    are wrapped so web/terminal operations cannot collide with gait writes.
# -----------------------------------------------------------------------------
class ThreadSafeDynamixelBus(ik.DynamixelBus):
    def __init__(self, port_name: str):
        super().__init__(port_name)
        self.io_lock = SERIAL_LOCK
        self.opened = False

    def open(self) -> bool:
        with self.io_lock:
            if self.opened:
                return True
            ok = super().open()
            self.opened = bool(ok)
            return ok

    def close(self):
        with self.io_lock:
            if not self.opened:
                return
            try:
                super().close()
            finally:
                self.opened = False

    def write1(self, motor_id: int, address: int, value: int) -> bool:
        with self.io_lock:
            return super().write1(motor_id, address, value)

    def write2(self, motor_id: int, address: int, value: int) -> bool:
        with self.io_lock:
            return super().write2(motor_id, address, value)

    def read1_once(self, motor_id: int, address: int) -> Optional[int]:
        with self.io_lock:
            return super().read1_once(motor_id, address)

    def read2_once(self, motor_id: int, address: int) -> Optional[int]:
        with self.io_lock:
            return super().read2_once(motor_id, address)

    def sync_write_positions(self, targets: dict[int, int]) -> bool:
        with self.io_lock:
            return super().sync_write_positions(targets)

    def sync_set_speeds(self, speed: int, motor_ids: Optional[list[int]] = None) -> bool:
        with self.io_lock:
            return super().sync_set_speeds(speed, motor_ids)

    def move_sync(self, targets: dict[int, int], speed: int):
        with self.io_lock:
            return super().move_sync(targets, speed)

    def move_many(self, targets: dict[int, int], speed: int):
        with self.io_lock:
            return super().move_many(targets, speed)

    def move_many_legacy(self, targets: dict[int, int], speed: int):
        with self.io_lock:
            return super().move_many_legacy(targets, speed)



# -----------------------------------------------------------------------------
# Web UI patching: keep the original IKControl look, only prefix API routes and
# inject the Serial / COM Port Switch panel.
# -----------------------------------------------------------------------------
def hexapod_html(runtime_obj: "HexapodRuntime") -> str:
    html = ik.WEB_HTML
    html = html.replace("'/api/", "'/hexapod/api/")
    html = html.replace('"/api/', '"/hexapod/api/')
    html = html.replace('`/api/', '`/hexapod/api/')

    # Fix original IKControl web key handler bug:
    # keydown ignores INPUT, but keyup did not. Typing commands containing S/A/D/W/Q/E
    # such as "ports" or "switch com" could trigger stopMove() from the global keyup handler.
    html = html.replace(
        "document.addEventListener('keyup',e=>{const k=e.key.toLowerCase();",
        "document.addEventListener('keyup',e=>{if(e.target&&e.target.tagName==='INPUT')return;const k=e.key.toLowerCase();",
    )
    html = html.replace(
        'onkeydown="if(event.key===\'Enter\') sendTerm()"',
        'onkeydown="if(event.key===\'Enter\'){event.preventDefault();sendTerm()}"',
    )

    panel = port_switch_panel_html(runtime_obj)
    # Place it between Terminal / Debug Output and Live Robot Status when possible.
    if '<div class="section statuspanel">' in html:
        html = html.replace('<div class="section statuspanel">', panel + '<div class="section statuspanel">', 1)
    elif '<div id="pills" class="statuscards"></div>' in html:
        html = html.replace('<div id="pills" class="statuscards"></div>', panel + '<div id="pills" class="statuscards"></div>', 1)
    else:
        html += panel
    return html


# -----------------------------------------------------------------------------
# Serial / COM port runtime switching
# -----------------------------------------------------------------------------
def _same_device_path(a: str, b: str) -> bool:
    """Best-effort comparison for /dev/ttyUSB0 vs /dev/serial/by-id/..."""
    try:
        return os.path.realpath(a) == os.path.realpath(b)
    except Exception:
        return str(a) == str(b)


def _add_port(rows: list[dict[str, Any]], seen: set[str], device: str, description: str = "", hwid: str = "") -> None:
    device = str(device or "").strip()
    if not device or device in seen:
        return
    seen.add(device)
    desc = str(description or "serial port").strip()
    hw = str(hwid or "").strip()
    label = f"{device}  —  {desc}" if desc else device
    if hw:
        label += f"  [{hw}]"
    rows.append({
        "device": device,
        "description": desc,
        "hwid": hw,
        "label": label,
        "detected": True,
    })


def list_available_serial_ports() -> list[dict[str, Any]]:
    """
    Fast, non-blocking port discovery.
    Prioritises stable Linux by-id paths first, then pyserial, then common glob paths.
    This function only lists device nodes; it does NOT open/test the motor bus.
    """
    rows: list[dict[str, Any]] = []
    seen: set[str] = set()

    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        _add_port(rows, seen, p, "stable by-id symlink")

    try:
        from serial.tools import list_ports  # type: ignore
        for p in list_ports.comports():
            _add_port(rows, seen, p.device, p.description or "serial port", p.hwid or "")
    except Exception:
        pass

    for pattern, desc in [
        ("/dev/ttyUSB*", "USB serial"),
        ("/dev/ttyACM*", "USB ACM serial"),
        ("/dev/ttyAMA*", "UART serial"),
        ("/dev/ttyS*", "hardware serial"),
    ]:
        for p in sorted(glob.glob(pattern)):
            _add_port(rows, seen, p, desc)

    return rows


def format_serial_ports_text(runtime_obj: "HexapodRuntime") -> str:
    ports = list_available_serial_ports()
    current = runtime_obj.port_name
    lines = [
        "===================================================",
        " SERIAL / COM PORTS",
        "===================================================",
        f"Current shared Dynamixel bus: {current}",
        "",
    ]
    if not ports:
        lines += [
            "No serial ports detected by list_ports/glob.",
            "You can still type an exact port manually, for example:",
            "  switch com /dev/ttyUSB0",
            "  switch com COM6",
        ]
    else:
        lines.append("Available ports:")
        for i, p in enumerate(ports, 1):
            device = p["device"]
            mark = "  <-- current" if device == current or _same_device_path(device, current) else ""
            lines.append(f"  {i}. {p['label']}{mark}")
    lines += [
        "",
        "Switch commands:",
        "  switch com <port>",
        "  switch port <port>",
        "  com <port>",
        "  port <port>",
        "",
        "Examples:",
        "  switch com /dev/ttyUSB1",
        "  switch com /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A40129VX-if00-port0",
        "  com COM6",
    ]
    return "\n".join(lines)


def log_serial_ports_to_web(runtime_obj: "HexapodRuntime") -> None:
    for line in format_serial_ports_text(runtime_obj).splitlines():
        ik.web_log(line)


def parse_serial_command(raw_cmd: str) -> tuple[str, str | None] | None:
    raw = (raw_cmd or "").strip()
    if not raw:
        return None
    parts = raw.split()
    low = [p.lower() for p in parts]

    if low[0] in ["ports", "coms", "serials", "serial", "porrs"]:
        return ("list", None)

    if low[0] in ["port", "com"]:
        if len(parts) == 1:
            return ("list", None)
        return ("switch", " ".join(parts[1:]).strip())

    if low[0] == "switch" and len(parts) >= 2 and low[1] in ["com", "port", "serial"]:
        if len(parts) < 3:
            return ("list", None)
        return ("switch", " ".join(parts[2:]).strip())

    return None


def execute_serial_command(runtime_obj: "HexapodRuntime", raw_cmd: str, web: bool = False) -> dict[str, Any] | None:
    parsed = parse_serial_command(raw_cmd)
    if parsed is None:
        return None

    action, target = parsed
    if action == "list":
        if web:
            ik.web_log(f"> {(raw_cmd or '').strip()}")
            log_serial_ports_to_web(runtime_obj)
        else:
            print(format_serial_ports_text(runtime_obj))
        return {"ok": True, "message": "Listed serial ports", "ports": list_available_serial_ports(), "current_port": runtime_obj.port_name}

    if action == "switch":
        if not target:
            msg = "Usage: switch com <port>  OR  com <port>"
            if web:
                ik.web_log(msg)
            else:
                print(msg)
            return {"ok": False, "message": msg}
        if web:
            ik.web_log(f"> {(raw_cmd or '').strip()}")
            ik.web_log(f"Switching shared Dynamixel bus to: {target}")
        else:
            print(f"Switching shared Dynamixel bus to: {target}")
        result = runtime_obj.switch_port(target)
        msg = result.get("message", "Port switch finished")
        if web:
            ik.web_log(msg)
        else:
            print(msg)
        return result

    return None


def port_switch_panel_html(runtime_obj: "HexapodRuntime") -> str:
    """Server-render the dropdown so it is usable even before JavaScript refresh runs."""
    ports = list_available_serial_ports()
    current = runtime_obj.port_name
    current_safe = html_escape(current)

    option_html: list[str] = []
    if current and not any(p["device"] == current for p in ports):
        option_html.append(f'<option value="{html_escape(current)}" selected>{html_escape(current)}  —  current/manual</option>')
    for p in ports:
        dev = p["device"]
        selected = " selected" if dev == current or _same_device_path(dev, current) else ""
        option_html.append(f'<option value="{html_escape(dev)}"{selected}>{html_escape(p["label"])}</option>')
    if not option_html:
        option_html.append('<option value="">No ports detected; type exact port below</option>')

    options = "\n".join(option_html)
    return f"""
<div class="section" id="combined-com-panel" style="border:1px solid #58a6ff;border-radius:14px;padding:16px;margin:18px 0;background:#0b1118">
  <h2 style="margin-top:0">Serial / COM Port Switch</h2>
  <div class="sub" style="margin-bottom:10px">Current shared Dynamixel bus: <b id="combined-com-current" style="color:#58a6ff">{current_safe}</b></div>
  <div style="display:flex;gap:10px;align-items:center;flex-wrap:wrap">
    <select id="combined-com-select" style="flex:1;min-width:280px;padding:12px;border-radius:12px;background:#05080d;color:#e6edf3;border:1px solid #8b949e;font-weight:800">
      {options}
    </select>
    <input id="combined-com-manual" placeholder="optional exact port e.g. /dev/ttyUSB1 or COM6" style="flex:1;min-width:270px;padding:12px;border-radius:12px;background:#05080d;color:#e6edf3;border:1px solid #30363d">
    <button class="btn small" type="button" onclick="combinedRefreshPorts(true)">Refresh COMs</button>
    <button class="btn small danger" type="button" onclick="combinedSwitchComFromPanel()">Switch COM</button>
  </div>
  <div id="combined-com-message" class="sub" style="margin-top:10px">Dropdown was filled by the server. Refresh only updates the list.</div>
  <div class="sub" style="margin-top:8px">Web terminal commands also work: <b>ports</b>, <b>port</b>, <b>switch com /dev/ttyUSB1</b>, <b>com COM6</b>.</div>
</div>
<script>
(function(){{
  function byId(id){{return document.getElementById(id);}}
  function setMsg(msg, bad){{var el=byId('combined-com-message'); if(el){{el.textContent=msg; el.style.color=bad?'#ffb4b4':'#8b949e';}}}}
  function terminalLog(msg){{var log=byId('log'); if(log){{log.textContent+=(log.textContent?'\\n':'')+msg; log.scrollTop=log.scrollHeight;}}}}
  function fillPorts(data){{
    var sel=byId('combined-com-select');
    var cur=byId('combined-com-current');
    if(!sel){{return;}}
    if(cur){{cur.textContent=data.current_port || '--';}}
    sel.innerHTML='';
    var ports=data.ports || [];
    if(!ports.length){{
      var opt=document.createElement('option'); opt.value=''; opt.textContent='No ports detected; type exact port manually'; sel.appendChild(opt);
    }} else {{
      for(var i=0;i<ports.length;i++){{
        var p=ports[i];
        var opt=document.createElement('option');
        opt.value=p.device || '';
        opt.textContent=p.label || p.device || '';
        if((p.device || '') === (data.current_port || '')){{opt.selected=true;}}
        sel.appendChild(opt);
      }}
    }}
    setMsg('COM list loaded: '+ports.length+' port(s).', false);
  }}
  window.combinedRefreshPorts = async function(showLog){{
    setMsg('Refreshing COM list...', false);
    try{{
      var r=await fetch('/system/ports?t='+Date.now(), {{cache:'no-store'}});
      var text=await r.text();
      var data=JSON.parse(text);
      if(!data.ok){{throw new Error(data.error || text);}}
      fillPorts(data);
      if(showLog){{terminalLog('[WEB] COM list refreshed. Current: '+(data.current_port || '--'));}}
    }}catch(e){{
      setMsg('COM refresh failed: '+e, true);
      if(showLog){{terminalLog('[WEB] COM refresh failed: '+e);}}
    }}
  }};
  window.combinedSwitchComFromPanel = async function(){{
    var manual=byId('combined-com-manual');
    var sel=byId('combined-com-select');
    var port=(manual && manual.value.trim()) || (sel && sel.value) || '';
    if(!port){{setMsg('Choose a port or type one manually.', true); return;}}
    if(!confirm('Switch shared Dynamixel bus to '+port+'?\\n\\nThis stops web motion and closes/reopens the serial bus.')){{return;}}
    setMsg('Switching to '+port+'...', false);
    try{{
      var r=await fetch('/system/port', {{method:'POST', headers:{{'Content-Type':'application/json'}}, body:JSON.stringify({{port:port}})}});
      var data=await r.json();
      if(!data.ok){{throw new Error(data.error || data.message || 'switch failed');}}
      setMsg(data.message || ('Switched to '+port), false);
      terminalLog('[WEB] '+(data.message || ('Switched COM to '+port)));
      await window.combinedRefreshPorts(false);
      setTimeout(function(){{ if(typeof refresh === 'function'){{refresh();}} }}, 400);
    }}catch(e){{
      setMsg('Switch failed: '+e, true);
      terminalLog('[WEB] Switch failed: '+e);
    }}
  }};
  function install(){{ setTimeout(function(){{ window.combinedRefreshPorts(false); }}, 200); }}
  if(document.readyState === 'loading'){{document.addEventListener('DOMContentLoaded', install);}} else {{install();}}
}})();
</script>
"""

# -----------------------------------------------------------------------------
# Hexapod controller lifecycle
# -----------------------------------------------------------------------------

class HexapodRuntime:
    def __init__(self, bus: ThreadSafeDynamixelBus, port_name: str) -> None:
        self.bus = bus
        self.port_name = port_name
        self._switch_lock = threading.RLock()

    def switch_port(self, new_port: str) -> dict[str, Any]:
        """Safely move the shared Dynamixel bus to another serial port."""
        new_port = str(new_port or "").strip()
        if not new_port:
            return {"ok": False, "message": "No port supplied", "error": "No port supplied"}

        with self._switch_lock:
            old_port = self.port_name
            if new_port == old_port:
                return {"ok": True, "message": f"Already using {new_port}", "current_port": self.port_name}

            old_bus = self.bus

            try:
                ik.web_stop_motion()
            except Exception:
                pass
            try:
                t = getattr(ik, "WEB_MOTION_THREAD", None)
                if t is not None and t.is_alive():
                    t.join(timeout=4.0)
            except Exception:
                pass

            with ik.WEB_BUSY_LOCK:
                try:
                    old_bus.close()
                except Exception as exc:
                    print(f"Old bus close warning during COM switch: {exc}")

                new_bus = ThreadSafeDynamixelBus(new_port)
                try:
                    ok = bool(new_bus.open())
                except Exception as exc:
                    ok = False
                    open_error = exc
                else:
                    open_error = None

                if not ok:
                    rollback_ok = False
                    try:
                        rollback_ok = bool(old_bus.open())
                    except Exception as rollback_exc:
                        print(f"Rollback to old port failed: {rollback_exc}")
                    self.bus = old_bus
                    self.port_name = old_port
                    ik.WEB_BUS = old_bus
                    msg = f"Failed to open {new_port}; stayed on {old_port}."
                    if open_error:
                        msg += f" Error: {open_error}"
                    if not rollback_ok:
                        msg += " WARNING: rollback reopen also failed."
                    return {"ok": False, "message": msg, "error": msg, "current_port": self.port_name}

                self.bus = new_bus
                self.port_name = new_port
                ik.WEB_BUS = new_bus

            return {"ok": True, "message": f"Switched Dynamixel bus: {old_port} -> {new_port}", "current_port": self.port_name}

    def shutdown(self) -> None:
        try:
            ik.web_stop_motion()
        except Exception:
            pass
        try:
            self.bus.close()
        except Exception as exc:
            print(f"Bus close warning: {exc}")


runtime: HexapodRuntime | None = None


def rt() -> HexapodRuntime:
    if runtime is None:
        raise RuntimeError("Hexapod runtime has not started.")
    return runtime



# -----------------------------------------------------------------------------
# Flask hexapod web server
# -----------------------------------------------------------------------------
def create_hexapod_web_app(runtime_obj: HexapodRuntime) -> Flask:
    app = Flask(__name__)
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    @app.after_request
    def no_cache(response):
        response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
        return response

    @app.get("/")
    def root():
        return redirect("/hexapod")


    @app.get("/system/state")
    def system_state():
        return jsonify({"ok": True, "current_port": runtime_obj.port_name})

    @app.get("/system/ports")
    def system_ports():
        return jsonify({
            "ok": True,
            "current_port": runtime_obj.port_name,
            "ports": list_available_serial_ports(),
        })

    @app.post("/system/port")
    def system_switch_port():
        try:
            payload = request.get_json(force=True) or {}
            port = str(payload.get("port", "")).strip()
            result = runtime_obj.switch_port(port)
            status = 200 if result.get("ok") else 400
            return jsonify(result), status
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc), "message": str(exc), "current_port": runtime_obj.port_name}), 400

    # ------------------------- Hexapod page + API -------------------------
    @app.get("/hexapod")
    def hexapod_page():
        return hexapod_html(runtime_obj)

    @app.get("/hexapod/api/state")
    def hex_state():
        return jsonify(ik.web_state(runtime_obj.bus))

    @app.post("/hexapod/api/command")
    def hex_command():
        payload = request.get_json(force=True) or {}
        raw_command = str(payload.get("command", ""))

        # Handle COM commands before IKControl sees them. This prevents commands like
        # "ports" or "switch com /dev/ttyUSB1" from becoming unknown IK commands.
        serial_result = execute_serial_command(runtime_obj, raw_command, web=True)
        if serial_result is not None:
            return jsonify(serial_result)

        with ik.WEB_BUSY_LOCK:
            return jsonify(ik.web_run_terminal_command(runtime_obj.bus, raw_command))

    @app.post("/hexapod/api/move/start")
    def hex_move_start():
        payload = request.get_json(force=True) or {}
        return jsonify(ik.web_start_motion(runtime_obj.bus, str(payload.get("direction", ""))))

    @app.post("/hexapod/api/move/stop")
    def hex_move_stop():
        ik.web_stop_motion()
        return jsonify({"ok": True, "message": "Stop requested"})

    @app.post("/hexapod/api/action/bodylevel")
    def hex_bodylevel():
        payload = request.get_json(force=True) or {}
        if ik.WEB_MOTION_THREAD and ik.WEB_MOTION_THREAD.is_alive():
            return jsonify({"ok": False, "message": f"Busy with {ik.WEB_CURRENT_MOTION}; bodylevel ignored"})
        if ik.WEB_CURRENT_MOTION not in ["idle", "blocked"]:
            return jsonify({"ok": False, "message": f"Not idle ({ik.WEB_CURRENT_MOTION}); bodylevel ignored"})
        with ik.WEB_BUSY_LOCK:
            mode = str(payload.get("mode", "set")).lower()
            if mode == "reset":
                ok = ik.action_body_level_reset(runtime_obj.bus)
            elif mode == "delta":
                ok = ik.action_body_level_delta(runtime_obj.bus, int(payload.get("delta", 0)))
            else:
                ok = ik.action_body_level_set(runtime_obj.bus, int(payload.get("level", 0)), True)
        return jsonify({"ok": bool(ok), "message": f"bodylevel {ik.BODY_HEIGHT_LEVEL}", "level": ik.BODY_HEIGHT_LEVEL})

    @app.post("/hexapod/api/action/pushup")
    def hex_pushup():
        payload = request.get_json(force=True) or {}
        if ik.WEB_MOTION_THREAD and ik.WEB_MOTION_THREAD.is_alive():
            return jsonify({"ok": False, "message": f"Busy with {ik.WEB_CURRENT_MOTION}; pushup ignored"})
        with ik.WEB_BUSY_LOCK:
            ok = ik.action_pushup_quick(runtime_obj.bus, str(payload.get("level", 1)))
        return jsonify({"ok": bool(ok), "message": f"pushup {payload.get('level', 1)}"})

    @app.post("/hexapod/api/action/liftall")
    def hex_liftall():
        payload = request.get_json(force=True) or {}
        if ik.WEB_MOTION_THREAD and ik.WEB_MOTION_THREAD.is_alive():
            return jsonify({"ok": False, "message": f"Busy with {ik.WEB_CURRENT_MOTION}; liftall ignored"})
        with ik.WEB_BUSY_LOCK:
            level = int(payload.get("level", 7))
            ok = ik.action_lift_all_quick(runtime_obj.bus, level)
        return jsonify({"ok": bool(ok), "message": f"liftall {level}"})

    return app



# -----------------------------------------------------------------------------
# Terminal hexapod mode
# -----------------------------------------------------------------------------
def print_hexapod_help() -> None:
    print("""
Hexapod terminal commands:
  h / help                  Show this help
  x / exit                  Quit safely

Hexapod commands:
  r, health, p, w, s, a, d, q, e, stop, speed all 25, ik, bodyik, walk forward 2...
  hex <command>             Force a command to IKControl, e.g. hex r

Serial / COM commands, available in web terminal and this terminal:
  ports / coms              List detected ports and current Dynamixel bus
  port / com                Show current port plus switch examples
  switch com <port>         Safely switch the Dynamixel bus
  switch port <port>        Same as switch com
  com <port>                Short alias, e.g. com COM6 or com /dev/ttyUSB1

Safety:
  stop / safe stop          Stop current web/hold motion
""")


def hexapod_terminal_loop(runtime_obj: HexapodRuntime) -> None:
    print("\n===================================================")
    print(" HEXAPOD TERMINAL MODE")
    print("===================================================")
    print("Commands go to IKControl by default. Vision commands are removed.")
    print_hexapod_help()

    while True:
        try:
            raw = input("\nhexapod > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting hexapod terminal.")
            break
        if not raw:
            continue
        low = raw.lower()
        try:
            if low in ["x", "exit", "quit"]:
                break
            if low in ["h", "help", "?"]:
                print_hexapod_help()
                continue
            serial_result = execute_serial_command(runtime_obj, raw, web=False)
            if serial_result is not None:
                continue
            if low in ["safe stop", "all stop", "both stop"]:
                ik.web_stop_motion()
                print("Stop requested for hexapod motion.")
                continue
            if low.startswith("hex "):
                raw = raw.split(maxsplit=1)[1]
            keep_running = ik.terminal_execute_command(runtime_obj.bus, raw)
            if not keep_running:
                break
        except Exception as exc:
            print(f"HEXAPOD COMMAND ERROR: {type(exc).__name__}: {exc}")


def choose_mode(default: str = "web") -> str:
    choice = input(f"Run mode [web/terminal] [{default}]: ").strip().lower()
    if not choice:
        return default
    if choice in ["w", "web"]:
        return "web"
    if choice in ["t", "terminal", "term"]:
        return "terminal"
    print("Unknown mode; using web.")
    return "web"



def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Hexapod-only IKControl launcher with runtime Serial/COM switching")
    p.add_argument("--mode", choices=["web", "terminal"], help="Run web UI or terminal mode")
    p.add_argument("--port", help="Dynamixel serial port, e.g. COM6 or /dev/ttyUSB0. If omitted, IKControl selector is used.")
    p.add_argument("--com", dest="port", help="Same as --port. Clearer Windows alias, e.g. --com COM4")
    p.add_argument("--no-browser", action="store_true", help="Do not auto-open browser in web mode")
    p.add_argument("--web-port", type=int, default=8000, help="Web UI port. Default: 8000")
    return p.parse_args()


def open_browser_later() -> None:
    time.sleep(1.0)
    webbrowser.open(f"http://{LOCAL_BROWSER_HOST}:{WEB_PORT}/hexapod")


def main() -> None:
    global runtime
    args = parse_args()
    global WEB_PORT
    WEB_PORT = int(args.web_port)
    mode = args.mode or choose_mode("web")
    selected_port = args.port or ik.choose_serial_port()

    bus = ThreadSafeDynamixelBus(selected_port)
    ik.WEB_BUS = bus
    if not bus.open():
        return

    ik.apply_web_startup_defaults()
    runtime = HexapodRuntime(bus, selected_port)
    atexit.register(runtime.shutdown)

    try:
        if mode == "web":
            app = create_hexapod_web_app(runtime)
            print("\n===================================================")
            print(" HEXAPOD WEB CONTROL")
            print("===================================================")
            print(f"Local on this Pi/PC: http://127.0.0.1:{WEB_PORT}/hexapod")
            print(f"LAN from laptop:     http://<PI-IP>:{WEB_PORT}/hexapod")
            print(f"Example:             http://<PI-IP>:{WEB_PORT}/hexapod")
            print(f"Listening on:        0.0.0.0:{WEB_PORT}")
            print("Hexapod-only mode: no vision page, no camera/model process, one COM bus owner.")
            print("Press Ctrl+C to stop safely.")
            if not args.no_browser:
                threading.Thread(target=open_browser_later, daemon=True).start()
            app.run(host=WEB_HOST, port=WEB_PORT, debug=False, threaded=True, use_reloader=False)
        else:
            hexapod_terminal_loop(runtime)
    finally:
        runtime.shutdown()


if __name__ == "__main__":
    main()
