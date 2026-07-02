#!/usr/bin/env python3
"""
导航命令监听器：订阅 /nav/command 话题，根据命令启动对应脚本。
"""

import json
import os
import signal
import subprocess
import sys
import time
import argparse
import threading
import websocket


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 命令 → 脚本映射
COMMAND_SCRIPTS = {
    "patrol": os.path.join(SCRIPT_DIR, "Task.py"),
    "goto": None,       # 预留：定点导航脚本
    "recharge": None,   # 预留：回充脚本
}


class NavCommander:
    def __init__(self, rosbridge_url: str, topic: str = "/nav/command"):
        self.url = rosbridge_url
        self.topic = topic
        self.ws = None
        self._running = True
        self.current_process: subprocess.Popen | None = None
        self.current_command: str | None = None

    def _on_open(self, ws):
        print(f"[commander] 已连接 rosbridge: {self.url}")
        ws.send(json.dumps({"op": "subscribe", "topic": self.topic}))
        print(f"[commander] 已订阅: {self.topic}")

    def _on_message(self, ws, raw: str):
        try:
            packet = json.loads(raw)
        except json.JSONDecodeError:
            return

        if packet.get("op") != "publish" or packet.get("topic") != self.topic:
            return

        msg = packet.get("msg", {})
        data_str = msg.get("data", "") if isinstance(msg, dict) else ""

        try:
            payload = json.loads(data_str)
        except (json.JSONDecodeError, TypeError):
            print(f"[commander] 无法解析消息: {data_str[:100]}")
            return

        cmd = payload.get("command", "")
        print(f"[commander] 收到命令: {cmd}")

        if cmd == "stop":
            self._stop_current()
        elif cmd in COMMAND_SCRIPTS:
            self._start_script(cmd)
        else:
            print(f"[commander] 未知命令: {cmd}")

    def _stop_current(self):
        if self.current_process and self.current_process.poll() is None:
            print(f"[commander] 停止当前脚本: {self.current_command}")
            self.current_process.send_signal(signal.SIGINT)
            try:
                self.current_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.current_process.kill()
                self.current_process.wait()
        self.current_process = None
        self.current_command = None
        print("[commander] 已停止")

    def _start_script(self, cmd: str):
        script_path = COMMAND_SCRIPTS.get(cmd)
        if not script_path:
            print(f"[commander] 脚本未配置: {cmd}")
            return

        # 先停旧脚本
        self._stop_current()

        print(f"[commander] 启动脚本: {script_path}")
        try:
            self.current_process = subprocess.Popen(
                ["python3", script_path],
                cwd=SCRIPT_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self.current_command = cmd

            # 后台读取输出
            def _read_output():
                if self.current_process and self.current_process.stdout:
                    for line in self.current_process.stdout:
                        print(f"[{cmd}] {line.rstrip()}")

            t = threading.Thread(target=_read_output, daemon=True)
            t.start()
        except Exception as e:
            print(f"[commander] 启动失败: {e}")

    def _on_error(self, ws, error):
        print(f"[commander] WS 错误: {error}")

    def _on_close(self, ws, code, msg):
        print(f"[commander] 断开 (code={code}), 3秒后重连...")
        if self._running:
            time.sleep(3)
            self.start()

    def start(self):
        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )
        t = threading.Thread(target=self.ws.run_forever, daemon=True)
        t.start()

    def stop(self):
        self._running = False
        self._stop_current()
        if self.ws:
            self.ws.close()


def main():
    parser = argparse.ArgumentParser(description="导航命令监听器")
    parser.add_argument("--url", default="ws://127.0.0.1:9090", help="rosbridge URL")
    parser.add_argument("--topic", default="/nav/command", help="命令话题")
    args = parser.parse_args()

    print(f"[commander] rosbridge: {args.url}")
    print(f"[commander] 订阅话题: {args.topic}")
    print(f"[commander] 命令映射: {json.dumps({k: os.path.basename(v) if v else '未配置' for k, v in COMMAND_SCRIPTS.items()}, indent=2)}")

    commander = NavCommander(args.url, args.topic)

    def shutdown(sig, frame):
        print("\n[commander] 退出...")
        commander.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    commander.start()

    while commander._running:
        time.sleep(1)


if __name__ == "__main__":
    main()
