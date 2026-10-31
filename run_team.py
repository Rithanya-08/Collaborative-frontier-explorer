#!/usr/bin/env python3
"""
Run both robots together from the laptop - one command.

Robot A's brain runs on its Raspberry Pi (started over SSH), Robot B's brain
runs here on the laptop and drives the ESP32 over MQTT. Afterwards Robot A's
map is fetched from the Pi and both maps are rendered to PNGs in maps/, plus
a shared_map_*.png that merges them (see merge_maps.py).

Before running:
- mosquitto running on this laptop (it starts with Windows)
- Robot B's ESP32 and Robot A's Pi powered and on the hotspot
- both robots on their start marks: B 50 cm to A's right, both facing the same way
- SSH key login to the Pi set up (ssh raspberry@Raspberry.local works without a password)

Usage (from the project folder):
    python run_team.py                     # 30-second run
    python run_team.py --minutes 3 --sync
    python run_team.py --planner rrt       # route planner: astar (default), rrt, prm, simple
While they run, a live team dashboard (app/terminal_dashboard.py) is shown
in this window under the robots' log lines; --plain shows only the log lines.
The browser dashboard (app/web_server.py) starts too, and its address is
shown to click; --no-dashboard skips it.
Ctrl+C stops both robots early; their maps are still saved.
"""

import argparse
import os
import socket
import subprocess
import sys
import threading
import time

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
PI = "raspberry@Raspberry.local"
PI_DIR = "~/robot"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]
# Code the Pi needs (firmware/, maps/ and app/ stay on the laptop)
SYNC_PATHS = ["main.py", "communication", "hardware", "navigation", "slam", "vision", "config", "scripts"]
# Pi CPU speed limit for runs (kHz; full speed is 2400000). On its power bank the
# Pi's 5 V input fell to 4.70 V with YOLO running at full speed - under the
# 4.75 V under-voltage limit. At 1.5 GHz it stayed at 4.91 V, the CPU current
# peaks halved, and one detection still takes only ~85 ms. Resets on reboot,
# so it is set again before every run.
PI_CPU_MAX_KHZ = 1500000
# Log lines worth showing live (use --verbose for everything)
KEY_WORDS = ("Pos:", "Route", "Sensor '", "Recognised", "Obstacle", "Map saved", "Run time", "Teammate", "Team link",
             "initialized successfully", "ERROR", "Error", "Traceback", "unavailable", "offline")


def port_open(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def robot_b_body_online(timeout=4.0):
    """Robot B's ESP32 publishes a retained online/offline status."""
    import paho.mqtt.client as mqtt
    status = {}
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="run-team-check")
    client.on_message = lambda c, u, m: status.setdefault("b", m.payload.decode())
    client.connect("localhost")
    client.subscribe("robot/B/status")
    client.loop_start()
    end = time.time() + timeout
    while "b" not in status and time.time() < end:
        time.sleep(0.1)
    client.loop_stop()
    client.disconnect()
    return status.get("b") == "online"


def laptop_ip():
    """This laptop's address on the robots' network, for opening the dashboard from another device."""
    for host in (PI.split("@")[1], "8.8.8.8"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((host, 9))  # UDP: nothing is sent
                return s.getsockname()[0]
        except OSError:
            continue
    return None


def ssh(command, **kwargs):
    return subprocess.run(SSH + [PI, command], capture_output=True, text=True, **kwargs)


def sync_code():
    """Copy the robot code to the Pi (tar over SSH)."""
    tar = subprocess.Popen(["tar", "--exclude=__pycache__", "-cf", "-", *SYNC_PATHS],
                           cwd=PROJECT_DIR, stdout=subprocess.PIPE)
    result = subprocess.run(SSH + [PI, f"mkdir -p {PI_DIR} && tar -xf - -C {PI_DIR}"],
                            stdin=tar.stdout, capture_output=True, text=True)
    tar.wait()
    if result.returncode != 0:
        sys.exit(f"Code sync to the Pi failed: {result.stderr.strip()}")
    print("✓ Code copied to the Pi")


def stream(name, proc, verbose, out=print):
    """Print a robot's log lines with a [A]/[B] prefix."""
    for line in proc.stdout:
        line = line.rstrip()
        if verbose or any(w in line for w in KEY_WORDS):
            out(f"[{name}] {line.split(' - ')[-1]}")


def wait_for(robots, live=None, monitor=None, started=None, run_seconds=0, links=()):
    """Wait for both robot processes, redrawing the live dashboard if shown."""
    from app.terminal_dashboard import render
    while any(p.poll() is None for p in robots):
        if live:
            rows = max(8, min(24, live.console.size.height - 22))
            live.update(render(monitor, run_seconds, started, map_rows=rows, links=links))
        time.sleep(0.5)
    if live:
        live.update(render(monitor, run_seconds, started, links=links))


def render_maps(b_started):
    """Fetch Robot A's newest map from the Pi and render both robots' maps."""
    sys.path.insert(0, PROJECT_DIR)
    import render_latest_map
    import merge_maps
    maps_dir = os.path.join(PROJECT_DIR, "maps")
    local_a = local_b = None

    remote = ssh(f"ls -t {PI_DIR}/maps/*.json 2>/dev/null | head -1").stdout.strip()
    if remote:
        path = os.path.join(maps_dir, "A_" + os.path.basename(remote))
        if subprocess.run(["scp", "-q", "-o", "BatchMode=yes", f"{PI}:{remote}", path]).returncode == 0:
            local_a = path
    b_maps = [os.path.join(maps_dir, f) for f in os.listdir(maps_dir)
              if f.startswith("map_") and f.endswith(".json")
              and os.path.getmtime(os.path.join(maps_dir, f)) >= b_started]
    if b_maps:
        local_b = max(b_maps, key=os.path.getmtime)

    for path in (local_a, local_b):
        if path:
            png = path[:-5] + ".png"
            render_latest_map.render(path, png)
            print(f"✓ Map image: {os.path.relpath(png, PROJECT_DIR)}")

    if local_a and local_b:
        png = merge_maps.merge_and_render(local_a, local_b)
        print(f"✓ Shared map image: {os.path.relpath(png, PROJECT_DIR)}")
    else:
        print("Shared map skipped - need both robots' maps")


def main():
    parser = argparse.ArgumentParser(description="Run Robot A and Robot B together")
    parser.add_argument("--minutes", type=float, default=0.5, help="run length (default 0.5 = 30 s)")
    parser.add_argument("--no-dashboard", action="store_true", help="don't start the browser dashboard")
    parser.add_argument("--dashboard", action="store_true", help=argparse.SUPPRESS)  # old flag; now the default
    parser.add_argument("--sync", action="store_true", help="copy the latest code to the Pi first")
    parser.add_argument("--planner", choices=["astar", "rrt", "prm", "simple"], default="astar",
                        help="route planner for both robots (default astar)")
    parser.add_argument("--no-vision", action="store_true", help="run Robot A without object recognition")
    parser.add_argument("--verbose", action="store_true", help="show every log line")
    parser.add_argument("--plain", action="store_true", help="log lines only, no live dashboard in this window")
    args = parser.parse_args()
    os.chdir(PROJECT_DIR)
    env = dict(os.environ, PYTHONIOENCODING="utf-8")

    # ---- checks ----
    if not port_open("localhost", 1883):
        sys.exit("✗ MQTT broker not running on this laptop - start the mosquitto service first")
    print("✓ MQTT broker running")
    if not robot_b_body_online():
        sys.exit("✗ Robot B's ESP32 is offline - check its power bank and that it joined the hotspot")
    print("✓ Robot B (ESP32) online")
    check = ssh("echo ok")
    if check.returncode != 0:
        sys.exit(f"✗ Can't reach Robot A's Pi ({PI}): {check.stderr.strip() or 'no answer'}")
    print("✓ Robot A (Pi) reachable")
    if ssh("pgrep -f 'python3 -u main[.]py' >/dev/null").returncode == 0:
        sys.exit("✗ Robot A is already running main.py - wait for it to finish first")
    if args.sync:
        sync_code()
    cap = ssh(f"echo {PI_CPU_MAX_KHZ} | sudo -n tee /sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq")
    if cap.returncode == 0:
        print(f"✓ Robot A's CPU limited to {PI_CPU_MAX_KHZ / 1e6:g} GHz to save power")
    else:
        print(f"! Couldn't limit Robot A's CPU speed: {cap.stderr.strip()}")

    dashboard = None
    dashboard_urls = []
    if not args.no_dashboard:
        if not port_open("localhost", 5000):
            dashboard = subprocess.Popen([sys.executable, "-m", "app.web_server"], env=env,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            for _ in range(20):
                if port_open("localhost", 5000):
                    break
                time.sleep(0.5)
        ip = laptop_ip()
        dashboard_urls = ["http://localhost:5000"] + ([f"http://{ip}:5000"] if ip else [])
        print(f"✓ Dashboard: {dashboard_urls[0]}" + (f"   (other devices: {dashboard_urls[1]})" if ip else ""))

    # ---- run ----
    live = monitor = None
    if not args.plain:
        from rich.live import Live
        from app.terminal_dashboard import TeamMonitor, render
        monitor = TeamMonitor("localhost", client_id="run-team-dashboard")
    a_cmd = f"cd {PI_DIR} && python3 -u main.py --robot a --minutes {args.minutes} --planner {args.planner}"
    if args.no_vision:
        a_cmd += " --no-vision"
    print(f"\n▶ Starting both robots for {args.minutes:g} min (Ctrl+C stops both, maps are still saved)\n")
    b_started = time.time()
    robot_a = subprocess.Popen(SSH + [PI, a_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, encoding="utf-8", errors="replace")
    robot_b = subprocess.Popen([sys.executable, "-u", "main.py", "--robot", "b", "--minutes", str(args.minutes),
                                "--planner", args.planner],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                               encoding="utf-8", errors="replace", env=env)
    if monitor:
        live = Live(render(monitor, args.minutes * 60, b_started, links=dashboard_urls), refresh_per_second=4)
        live.start()
        out = lambda line: live.console.print(line, markup=False, highlight=False)
    else:
        out = lambda line: print(line, flush=True)
    readers = [threading.Thread(target=stream, args=(n, p, args.verbose, out), daemon=True)
               for n, p in (("A", robot_a), ("B", robot_b))]
    for t in readers:
        t.start()
    try:
        wait_for((robot_a, robot_b), live, monitor, b_started, args.minutes * 60, dashboard_urls)
    except KeyboardInterrupt:
        # Ctrl+C reaches Robot B's process directly; closing SSH ends Robot A's
        # run (main.py saves its map on the hangup)
        print("\n■ Stopping both robots...")
        for p in (robot_b, robot_a):
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                p.terminate()
    for t in readers:
        t.join(timeout=2)
    if live:
        live.stop()  # the last dashboard frame stays on screen
        monitor.client.disconnect()

    print("\n▶ Run finished - collecting maps")
    try:
        render_maps(b_started)
    except Exception as e:
        print(f"Couldn't render maps: {e}")
    if dashboard:
        print(f"Dashboard still running at {' or '.join(dashboard_urls)} - Ctrl+C or close this window to stop it.")
        try:
            dashboard.wait()
        except KeyboardInterrupt:
            dashboard.terminate()


if __name__ == "__main__":
    main()
