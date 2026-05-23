#!/usr/bin/env python3
"""
Remote Parking Control Panel — 원격 주차 시뮬레이션 통합 제어 UI
"""

import os
import sys
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

WS_DIR = os.path.dirname(os.path.abspath(__file__))

SETUP_COMMANDS = (
    "source /opt/ros/humble/setup.bash && "
    "if [ -f {ws}/install/setup.bash ]; then source {ws}/install/setup.bash; fi"
).format(ws=WS_DIR)


def ros_cmd(cmd: str) -> str:
    return f"bash -c '{SETUP_COMMANDS} && {cmd}'"


class ProcessManager:
    def __init__(self):
        self._procs: dict[str, subprocess.Popen] = {}
        self._lock = threading.Lock()

    def start(self, name: str, cmd: str, on_line=None) -> bool:
        with self._lock:
            if name in self._procs and self._procs[name].poll() is None:
                return False
        full = ros_cmd(cmd)
        proc = subprocess.Popen(
            full, shell=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid, text=True, bufsize=1,
        )
        with self._lock:
            self._procs[name] = proc
        if on_line:
            t = threading.Thread(target=self._reader, args=(name, proc, on_line), daemon=True)
            t.start()
        return True

    def stop(self, name: str):
        with self._lock:
            proc = self._procs.pop(name, None)
        if proc and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass

    def is_running(self, name: str) -> bool:
        with self._lock:
            proc = self._procs.get(name)
            return proc is not None and proc.poll() is None

    def stop_all(self):
        with self._lock:
            names = list(self._procs.keys())
        for n in names:
            self.stop(n)

    def _reader(self, name, proc, on_line):
        try:
            for line in proc.stdout:
                on_line(f"[{name}] {line}")
        except Exception:
            pass

    def run_once(self, cmd: str, on_line=None, on_done=None):
        def _run():
            full = ros_cmd(cmd)
            proc = subprocess.Popen(
                full, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
            if on_line:
                for line in proc.stdout:
                    on_line(line)
            proc.wait()
            if on_done:
                on_done(proc.returncode)
        t = threading.Thread(target=_run, daemon=True)
        t.start()


class StatusMonitor:
    def __init__(self, on_status):
        self._on_status = on_status
        self._running = False
        self._proc = None

    def start(self):
        if self._running:
            return
        self._running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def stop(self):
        self._running = False
        if self._proc and self._proc.poll() is None:
            try:
                self._proc.terminate()
            except Exception:
                pass

    def _loop(self):
        while self._running:
            try:
                full = ros_cmd("ros2 topic echo /remote_parking/status std_msgs/msg/String --once --no-arr")
                self._proc = subprocess.Popen(
                    full, shell=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True, bufsize=1,
                )
                out, _ = self._proc.communicate(timeout=3)
                for line in out.splitlines():
                    line = line.strip()
                    if line.startswith("data:"):
                        state = line.split(":", 1)[1].strip().strip("'\"")
                        self._on_status(state)
                        break
            except subprocess.TimeoutExpired:
                if self._proc:
                    self._proc.kill()
                    self._proc.wait()
            except Exception:
                pass
            time.sleep(0.8)


class RemoteParkingUI:
    BG = "#1e1e2e"
    BG2 = "#2a2a3c"
    FG = "#cdd6f4"
    ACCENT = "#89b4fa"
    GREEN = "#a6e3a1"
    RED = "#f38ba8"
    YELLOW = "#f9e2af"
    ORANGE = "#fab387"
    SURFACE = "#313244"
    OVERLAY = "#45475a"

    STATE_COLORS = {
        "IDLE": "#a6adc8",
        "LIMO1_EVADE": "#89b4fa",
        "WAIT_FOR_SELECTION": "#f9e2af",
        "LIMO2_EXIT_INIT": "#fab387",
        "LIMO2_EXITING": "#fab387",
        "LIMO1_SCAN": "#cba6f7",
        "LIMO1_REPARK": "#89b4fa",
        "FINISH": "#a6e3a1",
        "ABORT": "#f38ba8",
    }

    def __init__(self):
        self.pm = ProcessManager()
        self.root = tk.Tk()
        self.root.title("Remote Parking Control Panel")
        self.root.configure(bg=self.BG)
        self.root.geometry("980x780")
        self.root.minsize(800, 600)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=self.BG)
        style.configure("Card.TFrame", background=self.BG2)
        style.configure("TLabel", background=self.BG, foreground=self.FG, font=("Noto Sans KR", 10))
        style.configure("Header.TLabel", background=self.BG, foreground=self.FG, font=("Noto Sans KR", 13, "bold"))
        style.configure("State.TLabel", background=self.BG2, foreground=self.GREEN, font=("Noto Sans KR", 20, "bold"))
        style.configure("Step.TLabel", background=self.BG2, foreground=self.ACCENT, font=("Noto Sans KR", 9))

        self._build_ui()
        self._status_var = "---"
        self.monitor = StatusMonitor(self._on_status_update)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _card(self, parent, **kw):
        f = ttk.Frame(parent, style="Card.TFrame", padding=12, **kw)
        return f

    def _btn(self, parent, text, command, color=None, width=18):
        color = color or self.ACCENT
        b = tk.Button(
            parent, text=text, command=command,
            bg=color, fg="#1e1e2e", activebackground=color, activeforeground="#1e1e2e",
            font=("Noto Sans KR", 10, "bold"), relief="flat", cursor="hand2",
            width=width, pady=6,
        )
        return b

    def _build_ui(self):
        main = ttk.Frame(self.root, style="TFrame", padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        # === Title ===
        title_frame = ttk.Frame(main, style="TFrame")
        title_frame.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(title_frame, text="Remote Parking Control Panel",
                  font=("Noto Sans KR", 16, "bold"), foreground=self.ACCENT,
                  background=self.BG).pack(side=tk.LEFT)
        ttk.Label(title_frame, text="ROS2 Humble + Gazebo",
                  font=("Noto Sans KR", 9), foreground=self.OVERLAY,
                  background=self.BG).pack(side=tk.RIGHT, pady=4)

        # === Top: Status + Steps ===
        top = ttk.Frame(main, style="TFrame")
        top.pack(fill=tk.X, pady=(0, 8))

        # -- Status Card --
        status_card = self._card(top)
        status_card.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        ttk.Label(status_card, text="FSM State", style="Header.TLabel",
                  background=self.BG2).pack(anchor=tk.W)
        self.state_label = ttk.Label(status_card, text="---", style="State.TLabel")
        self.state_label.pack(pady=8)
        self.state_desc = ttk.Label(status_card, text="", background=self.BG2,
                                    foreground=self.FG, font=("Noto Sans KR", 9),
                                    wraplength=200)
        self.state_desc.pack()

        # -- Steps Card --
        steps_card = self._card(top)
        steps_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        ttk.Label(steps_card, text="Mission Flow", style="Header.TLabel",
                  background=self.BG2).pack(anchor=tk.W, pady=(0, 6))

        flow_frame = ttk.Frame(steps_card, style="Card.TFrame")
        flow_frame.pack(fill=tk.X)

        steps = [
            ("1", "Gazebo\nServer", "IDLE"),
            ("2", "Spawn\nRobots", "IDLE"),
            ("3", "Mission\nStart", "LIMO1_EVADE"),
            ("4", "Car\nSelect", "WAIT_FOR_SELECTION"),
            ("5", "Exit &\nScan", "LIMO2_EXITING"),
            ("6", "Repark", "LIMO1_REPARK"),
            ("7", "Finish", "FINISH"),
        ]
        self.step_indicators = []
        for i, (num, label, _) in enumerate(steps):
            sf = ttk.Frame(flow_frame, style="Card.TFrame")
            sf.pack(side=tk.LEFT, padx=4, expand=True)
            circ = tk.Label(sf, text=num, bg=self.OVERLAY, fg=self.FG,
                            font=("Noto Sans KR", 10, "bold"), width=3, height=1)
            circ.pack()
            lbl = tk.Label(sf, text=label, bg=self.BG2, fg=self.FG,
                           font=("Noto Sans KR", 8), justify=tk.CENTER)
            lbl.pack(pady=2)
            self.step_indicators.append((circ, lbl, _))
            if i < len(steps) - 1:
                arrow = tk.Label(flow_frame, text="→", bg=self.BG2, fg=self.OVERLAY,
                                 font=("Noto Sans KR", 14))
                arrow.pack(side=tk.LEFT)

        # === Middle: Control Buttons ===
        mid = ttk.Frame(main, style="TFrame")
        mid.pack(fill=tk.X, pady=(0, 8))

        # -- Launch Section --
        launch_card = self._card(mid)
        launch_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 4))
        ttk.Label(launch_card, text="1. Launch", style="Header.TLabel",
                  background=self.BG2).pack(anchor=tk.W, pady=(0, 6))

        btn_row1 = ttk.Frame(launch_card, style="Card.TFrame")
        btn_row1.pack(fill=tk.X, pady=2)
        self.btn_gazebo = self._btn(btn_row1, "Gazebo Start", self._start_gazebo, self.GREEN)
        self.btn_gazebo.pack(side=tk.LEFT, padx=(0, 4))
        self.btn_gazebo_stop = self._btn(btn_row1, "Gazebo Stop", self._stop_gazebo, self.RED)
        self.btn_gazebo_stop.pack(side=tk.LEFT)

        btn_row2 = ttk.Frame(launch_card, style="Card.TFrame")
        btn_row2.pack(fill=tk.X, pady=2)
        self.btn_spawn = self._btn(btn_row2, "Spawn Robots", self._spawn_robots, self.ACCENT)
        self.btn_spawn.pack(side=tk.LEFT, padx=(0, 4))
        self.btn_spawn_stop = self._btn(btn_row2, "Spawn Stop", self._stop_spawn, self.RED)
        self.btn_spawn_stop.pack(side=tk.LEFT)

        self.gazebo_status = tk.Label(launch_card, text="  Gazebo: Stopped",
                                      bg=self.BG2, fg=self.RED, font=("Noto Sans KR", 9),
                                      anchor=tk.W)
        self.gazebo_status.pack(fill=tk.X, pady=(4, 0))
        self.spawn_status = tk.Label(launch_card, text="  Nodes: Stopped",
                                     bg=self.BG2, fg=self.RED, font=("Noto Sans KR", 9),
                                     anchor=tk.W)
        self.spawn_status.pack(fill=tk.X)

        # -- Mission Section --
        mission_card = self._card(mid)
        mission_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 4))
        ttk.Label(mission_card, text="2. Mission", style="Header.TLabel",
                  background=self.BG2).pack(anchor=tk.W, pady=(0, 6))

        self.btn_start = self._btn(mission_card, "Start Mission", self._start_mission, self.GREEN, width=30)
        self.btn_start.pack(fill=tk.X, pady=2)

        ttk.Label(mission_card, text="Select Exit Car:", style="Step.TLabel",
                  background=self.BG2).pack(anchor=tk.W, pady=(8, 2))
        car_row = ttk.Frame(mission_card, style="Card.TFrame")
        car_row.pack(fill=tk.X, pady=2)
        self.btn_a2 = self._btn(car_row, "A2", lambda: self._select_car("a2"), self.YELLOW, width=8)
        self.btn_a2.pack(side=tk.LEFT, padx=(0, 4))
        self.btn_a3 = self._btn(car_row, "A3", lambda: self._select_car("a3"), self.YELLOW, width=8)
        self.btn_a3.pack(side=tk.LEFT, padx=(0, 4))
        self.btn_a4 = self._btn(car_row, "A4", lambda: self._select_car("a4"), self.YELLOW, width=8)
        self.btn_a4.pack(side=tk.LEFT)

        # -- Reset Section --
        reset_card = self._card(mid)
        reset_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 0))
        ttk.Label(reset_card, text="3. Reset", style="Header.TLabel",
                  background=self.BG2).pack(anchor=tk.W, pady=(0, 6))

        self.btn_reset = self._btn(reset_card, "Reset (Teleport)", self._reset_teleport, self.ORANGE, width=22)
        self.btn_reset.pack(fill=tk.X, pady=2)
        self.btn_reset_full = self._btn(reset_card, "Reset (Respawn)", self._reset_respawn, self.RED, width=22)
        self.btn_reset_full.pack(fill=tk.X, pady=2)
        self.btn_stop_all = self._btn(reset_card, "STOP ALL", self._stop_all_procs, self.RED, width=22)
        self.btn_stop_all.pack(fill=tk.X, pady=(8, 2))

        # === Bottom: Log Output ===
        log_card = self._card(main)
        log_card.pack(fill=tk.BOTH, expand=True)

        log_header = ttk.Frame(log_card, style="Card.TFrame")
        log_header.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(log_header, text="Log", style="Header.TLabel",
                  background=self.BG2).pack(side=tk.LEFT)
        clear_btn = self._btn(log_header, "Clear", self._clear_log, self.OVERLAY, width=6)
        clear_btn.pack(side=tk.RIGHT)

        self.log_text = scrolledtext.ScrolledText(
            log_card, height=12, wrap=tk.WORD,
            bg="#181825", fg=self.FG, insertbackground=self.FG,
            font=("Consolas", 9), relief="flat", borderwidth=0,
            state=tk.DISABLED,
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.tag_configure("info", foreground=self.FG)
        self.log_text.tag_configure("success", foreground=self.GREEN)
        self.log_text.tag_configure("warn", foreground=self.YELLOW)
        self.log_text.tag_configure("error", foreground=self.RED)
        self.log_text.tag_configure("cmd", foreground=self.ACCENT)

        self._update_proc_status()

    def _log(self, text, tag="info"):
        def _do():
            self.log_text.configure(state=tk.NORMAL)
            ts = time.strftime("%H:%M:%S")
            self.log_text.insert(tk.END, f"[{ts}] {text}", tag)
            if not text.endswith("\n"):
                self.log_text.insert(tk.END, "\n")
            self.log_text.see(tk.END)
            self.log_text.configure(state=tk.DISABLED)
        self.root.after(0, _do)

    def _clear_log(self):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # === Actions ===
    def _start_gazebo(self):
        if self.pm.is_running("gazebo"):
            self._log("Gazebo is already running.", "warn")
            return
        self._log("Starting Gazebo world server (gui:=true)...", "cmd")
        self.pm.start(
            "gazebo",
            "ros2 launch remote_parking_world world_server.launch.py gui:=true",
            on_line=lambda l: self._log(l.rstrip()),
        )
        self.monitor.start()

    def _stop_gazebo(self):
        self._log("Stopping Gazebo...", "warn")
        self.pm.stop("gazebo")
        self.pm.stop("spawn")

    def _spawn_robots(self):
        if not self.pm.is_running("gazebo"):
            self._log("Gazebo is not running! Start Gazebo first.", "error")
            return
        if self.pm.is_running("spawn"):
            self._log("Nodes already running. Stop first or reset.", "warn")
            return
        self._log("Spawning robots + starting nodes...", "cmd")
        self.pm.start(
            "spawn",
            "ros2 launch remote_parking_world spawn_robots.launch.py",
            on_line=lambda l: self._log(l.rstrip()),
        )

    def _stop_spawn(self):
        self._log("Stopping spawn/nodes...", "warn")
        self.pm.stop("spawn")

    def _start_mission(self):
        self._log("Calling /start_remote_parking service...", "cmd")
        def on_line(l):
            self._log(l.rstrip())
        def on_done(rc):
            if rc == 0:
                self._log("Mission started successfully!", "success")
            else:
                self._log("Mission start failed.", "error")
        self.pm.run_once(
            "ros2 service call /start_remote_parking std_srvs/srv/Trigger '{}'",
            on_line=on_line, on_done=on_done,
        )

    def _select_car(self, car: str):
        self._log(f"Selecting exit car: {car.upper()}", "cmd")
        def on_line(l):
            self._log(l.rstrip())
        def on_done(rc):
            if rc == 0:
                self._log(f"Car {car.upper()} selected!", "success")
            else:
                self._log(f"Car {car.upper()} selection failed (service).", "error")
        self.pm.run_once(
            f"ros2 service call /select_exit_car/{car} std_srvs/srv/Trigger '{{}}'",
            on_line=on_line, on_done=on_done,
        )

    def _reset_teleport(self):
        self._log("Resetting robots (teleport)...", "cmd")
        script = os.path.join(WS_DIR, "src", "remote_parking_world", "scripts", "reset_robots.sh")
        self.pm.run_once(
            f"bash {script} teleport",
            on_line=lambda l: self._log(l.rstrip()),
            on_done=lambda rc: self._log("Reset complete." if rc == 0 else "Reset failed.", "success" if rc == 0 else "error"),
        )

    def _reset_respawn(self):
        self._log("Resetting robots (respawn — slower)...", "warn")
        script = os.path.join(WS_DIR, "src", "remote_parking_world", "scripts", "reset_robots.sh")
        self.pm.run_once(
            f"bash {script} respawn",
            on_line=lambda l: self._log(l.rstrip()),
            on_done=lambda rc: self._log("Reset complete." if rc == 0 else "Reset failed.", "success" if rc == 0 else "error"),
        )

    def _stop_all_procs(self):
        self._log("STOPPING ALL PROCESSES...", "error")
        self.pm.stop_all()
        self._log("All processes stopped.", "warn")

    # === Status Updates ===
    def _on_status_update(self, state: str):
        self.root.after(0, self._update_state_display, state)

    STATE_DESCS = {
        "IDLE": "Waiting for mission start.",
        "LIMO1_EVADE": "LIMO1 reversing to clear double-parked position.",
        "WAIT_FOR_SELECTION": "Waiting for exit car selection (A2/A3/A4).",
        "LIMO2_EXIT_INIT": "Generating exit waypoints for selected car.",
        "LIMO2_EXITING": "Selected car exiting via Pure Pursuit.",
        "LIMO1_SCAN": "Scanning for empty slot with LiDAR.",
        "LIMO1_REPARK": "LIMO1 reparking via Hybrid A*.",
        "FINISH": "Mission complete!",
        "ABORT": "Mission aborted.",
    }

    def _update_state_display(self, state: str):
        if state == self._status_var:
            return
        self._status_var = state
        color = self.STATE_COLORS.get(state, self.FG)
        self.state_label.configure(text=state, foreground=color)
        desc = self.STATE_DESCS.get(state, "")
        self.state_desc.configure(text=desc)
        self._log(f"State: {state}", "success" if state == "FINISH" else "info")
        self._highlight_step(state)

    def _highlight_step(self, state: str):
        state_to_step = {
            "IDLE": -1,
            "LIMO1_EVADE": 2,
            "WAIT_FOR_SELECTION": 3,
            "LIMO2_EXIT_INIT": 4,
            "LIMO2_EXITING": 4,
            "LIMO1_SCAN": 4,
            "LIMO1_REPARK": 5,
            "FINISH": 6,
            "ABORT": -1,
        }
        active = state_to_step.get(state, -1)
        for i, (circ, lbl, _) in enumerate(self.step_indicators):
            if i == active:
                circ.configure(bg=self.GREEN, fg="#1e1e2e")
            elif i < active:
                circ.configure(bg=self.ACCENT, fg="#1e1e2e")
            else:
                circ.configure(bg=self.OVERLAY, fg=self.FG)

    def _update_proc_status(self):
        gz = self.pm.is_running("gazebo")
        sp = self.pm.is_running("spawn")
        self.gazebo_status.configure(
            text=f"  Gazebo: {'Running' if gz else 'Stopped'}",
            fg=self.GREEN if gz else self.RED,
        )
        self.spawn_status.configure(
            text=f"  Nodes: {'Running' if sp else 'Stopped'}",
            fg=self.GREEN if sp else self.RED,
        )
        self.root.after(2000, self._update_proc_status)

    def _on_close(self):
        self.monitor.stop()
        self.pm.stop_all()
        self.root.destroy()

    def run(self):
        self._log("Remote Parking Control Panel started.", "success")
        self._log(f"Workspace: {WS_DIR}", "info")
        self._log("Step 1: Click 'Gazebo Start' to launch the world.", "info")
        self._log("Step 2: Click 'Spawn Robots' after Gazebo is ready.", "info")
        self._log("Step 3: Click 'Start Mission' to begin.", "info")
        self.root.mainloop()


if __name__ == "__main__":
    app = RemoteParkingUI()
    app.run()
