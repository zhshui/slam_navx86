#!/usr/bin/env python3
"""
桥接脚本：从 rosbridge WebSocket 订阅 /map/topology/update，
将拓扑点位转换为 Task.py 可读取的 JSON 格式，写入 data/ 目录。
"""

import json
import math
import os
import sys
import time
import argparse
import signal
import websocket


def yaw_to_quaternion(yaw: float):
    """yaw 角转四元数 (仅绕 Z 轴旋转)"""
    half = yaw / 2.0
    return {
        "ori_x": 0.0,
        "ori_y": 0.0,
        "ori_z": math.sin(half),
        "ori_w": math.cos(half),
    }


def topology_to_task_points(topology_msg: dict) -> list[dict]:
    """
    将拓扑地图消息转换为 Task.py 的 JSON 格式列表。
    
    Web 格式:
    {
      "points": [{"name": "NAV_POINT_0", "x": 1.0, "y": 2.0, "theta": 0.5, "type": 0}],
      "routes": [{"from_point": "A", "to_point": "B", "route_info": {...}}]
    }
    
    Task.py 格式:
    {
      "order": 0,
      "robot_pose": {"pos_x":..., "pos_y":..., "pos_z":0, "ori_x":..., "ori_y":..., "ori_z":..., "ori_w":...},
      "option": {"even_low_speed": false, ...}
    }
    """
    points = topology_msg.get("points", [])
    routes = topology_msg.get("routes", [])

    if not points:
        return []

    # 根据 routes 确定遍历顺序：从第一条 route 的 from_point 开始，按路线顺序排列
    ordered_names = []
    if routes:
        visited = set()
        for route in routes:
            if route["from_point"] not in visited:
                ordered_names.append(route["from_point"])
                visited.add(route["from_point"])
            if route["to_point"] not in visited:
                ordered_names.append(route["to_point"])
                visited.add(route["to_point"])
        # 追加未出现在 routes 中的孤立点
        for p in points:
            if p["name"] not in visited:
                ordered_names.append(p["name"])
    else:
        # 无路线时按原始顺序
        ordered_names = [p["name"] for p in points]

    # 建立 name → point 映射
    point_map = {p["name"]: p for p in points}

    task_points = []
    for order, name in enumerate(ordered_names):
        pt = point_map.get(name)
        if not pt:
            continue
        quat = yaw_to_quaternion(pt.get("theta", 0.0))
        task_points.append({
            "order": order,
            "robot_pose": {
                "pos_x": pt["x"],
                "pos_y": pt["y"],
                "pos_z": 0.0,
                **quat,
            },
            "option": {
                "even_low_speed": False,
                "even_medium_speed": False,
                "uneven_high_step": False,
            },
        })
    return task_points


def save_task_points(task_points: list[dict], data_dir: str):
    """清空旧文件，写入新的 task point JSON 文件."""
    os.makedirs(data_dir, exist_ok=True)

    # 清理旧文件
    for f in os.listdir(data_dir):
        if f.endswith(".json"):
            os.remove(os.path.join(data_dir, f))

    # 写入新文件
    for tp in task_points:
        order = tp["order"]
        filepath = os.path.join(data_dir, f"{order}.json")
        with open(filepath, "w") as fp:
            json.dump(tp, fp, indent=4)
        print(f"[topology_bridge] 写入: {filepath}  (x={tp['robot_pose']['pos_x']:.3f}, y={tp['robot_pose']['pos_y']:.3f})")

    print(f"[topology_bridge] 共写入 {len(task_points)} 个任务点")


class TopologyBridge:
    def __init__(self, rosbridge_url: str, data_dir: str, topic: str = "/map/topology/update", verbose: bool = False):
        self.url = rosbridge_url
        self.data_dir = data_dir
        self.topic = topic
        self.verbose = verbose
        self.ws: websocket.WebSocketApp | None = None
        self._running = True

    def _on_open(self, ws):
        print(f"[topology_bridge] 已连接 rosbridge: {self.url}")
        # 订阅拓扑更新话题 (不指定 type，让 rosbridge 透传原始消息)
        sub_msg = json.dumps({
            "op": "subscribe",
            "topic": self.topic,
        })
        ws.send(sub_msg)
        print(f"[topology_bridge] 已订阅: {self.topic} (无类型限制)")

    def _on_message(self, ws, raw: str):
        try:
            packet = json.loads(raw)
        except json.JSONDecodeError:
            return

        op = packet.get("op", "")
        topic = packet.get("topic", "")

        # 调试：打印收到的操作类型
        if self.verbose:
            print(f"[topology_bridge] 收到: op={op}, topic={topic}")
        elif op == "publish":
            msg = packet.get("msg", {})
            keys = list(msg.keys()) if isinstance(msg, dict) else 'N/A'
            print(f"[topology_bridge] 收到 publish: topic={topic}, msg_keys={keys}")

        if packet.get("op") != "publish" or packet.get("topic") != self.topic:
            return

        msg = packet.get("msg", {})
        if self.verbose:
            print(f"[topology_bridge] raw msg type={type(msg).__name__}, keys={list(msg.keys()) if isinstance(msg, dict) else 'N/A'}")

        # 兼容两种格式:
        # 1. std_msgs/String: msg = {"data": "{\"points\": [...]}"}
        # 2. 直接 topology 对象: msg = {"points": [...], "routes": [...]}
        topology = msg
        if isinstance(msg, dict) and "data" in msg and isinstance(msg["data"], str):
            try:
                topology = json.loads(msg["data"])
            except json.JSONDecodeError:
                print(f"[topology_bridge] 无法解析 data 字段: {msg['data'][:100]}")
                return

        print(f"[topology_bridge] 收到拓扑更新, 点位数量: {len(topology.get('points', []))}")

        task_points = topology_to_task_points(topology)
        if task_points:
            save_task_points(task_points, self.data_dir)
        else:
            print("[topology_bridge] 无有效点位，跳过")

    def _on_error(self, ws, error):
        print(f"[topology_bridge] WebSocket 错误: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        print(f"[topology_bridge] 连接断开 (code={close_status_code}), 3秒后重连...")
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
        # 在独立线程中运行
        import threading
        t = threading.Thread(target=self.ws.run_forever, daemon=True)
        t.start()

    def stop(self):
        self._running = False
        if self.ws:
            self.ws.close()


def send_test_message(url: str, topic: str):
    """发送测试拓扑消息，用于调试"""
    test_msg = {
        "op": "publish",
        "topic": topic,
        "msg": {
            "map_name": "test",
            "points": [
                {"name": "TEST_POINT_0", "x": 1.0, "y": 2.0, "theta": 0.0, "type": 0},
                {"name": "TEST_POINT_1", "x": 3.0, "y": 4.0, "theta": 1.57, "type": 0},
            ],
            "routes": [
                {"from_point": "TEST_POINT_0", "to_point": "TEST_POINT_1",
                 "route_info": {"controller": "FollowPath", "goal_checker": "general_goal_checker", "speed_limit": 1.0}},
            ],
        },
    }
    try:
        ws = websocket.create_connection(url, timeout=3)
        ws.send(json.dumps(test_msg))
        print(f"[test] 已发送测试消息到 {topic}")
        ws.close()
        return True
    except Exception as e:
        print(f"[test] 发送失败: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="拓扑地图桥接：rosbridge -> Task.py JSON")
    parser.add_argument("--url", default="ws://127.0.0.1:9090", help="rosbridge WebSocket URL")
    parser.add_argument("--topic", default="/map/topology/update", help="拓扑话题名")
    parser.add_argument("--data-dir", default=None, help="Task.py data 目录 (默认: 本脚本同级的 ../data)")
    parser.add_argument("--test", action="store_true", help="发送测试消息后退出")
    parser.add_argument("--verbose", action="store_true", help="打印所有收到的消息")
    args = parser.parse_args()

    if args.data_dir:
        data_dir = os.path.abspath(args.data_dir)
    else:
        data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data"))

    print(f"[topology_bridge] rosbridge URL: {args.url}")
    print(f"[topology_bridge] 订阅话题:   {args.topic}")
    print(f"[topology_bridge] 输出目录:   {data_dir}")

    if args.test:
        send_test_message(args.url, args.topic)
        print("[test] 测试消息已发送，请检查 bridge 是否收到")
        return

    bridge = TopologyBridge(args.url, data_dir, args.topic, verbose=args.verbose)

    def shutdown(sig, frame):
        print("\n[topology_bridge] 收到退出信号")
        bridge.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    bridge.start()

    # 保持主线程存活
    while bridge._running:
        time.sleep(1)


if __name__ == "__main__":
    main()
