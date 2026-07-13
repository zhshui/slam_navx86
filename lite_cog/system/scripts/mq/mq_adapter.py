#!/usr/bin/env python3
"""
MQ Adapter v3: RabbitMQ/MQTT ↔ ROS 导航桥接
支持 AMQP (pika) 和 MQTT (paho-mqtt)，通过 MQ_TYPE 环境变量切换
"""
import json, os, sqlite3, subprocess, threading, time, uuid, signal, math, yaml
from datetime import datetime, timezone
import urllib.request
from typing import Optional

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from std_msgs.msg import String as RosString

# ====== 默认环境变量（可通过 export 覆盖） ======
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# mq_adapter.py → mq/ → scripts/ → system/ → lite_cog/ → go2_nav/
NAV_ROOT = os.environ.get("NAV_ROOT", os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))))

MQ_TYPE = os.environ.get("MQ_TYPE", "mqtt")                 # rabbitmq | mqtt
MQ_HOST = os.environ.get("MQ_HOST", "49.233.183.203")
MQ_PORT = int(os.environ.get("MQ_PORT", "1883"))
MQ_USER = os.environ.get("MQ_USER", "server")
MQ_PASS = os.environ.get("MQ_PASS", "5bP!8aS3$kD7vF2&")
MQ_VHOST = os.environ.get("MQ_VHOST", "/")
MQ_EXCHANGE = os.environ.get("MQ_EXCHANGE", "nav.exchange")
MQ_CLIENT_ID = os.environ.get("MQ_CLIENT_ID", "B42D1000P3499GGW")
GATEWAY_DIR = os.environ.get("GATEWAY_DIR", os.path.join(NAV_ROOT, "ros_web_gui_app/gateway"))
DB_PATH = os.path.join(GATEWAY_DIR, "data", "nav_web.sqlite")
SWITCH_HOOK = os.environ.get("MAP_SWITCH_HOOK", os.path.join(NAV_ROOT, "lite_cog/system/scripts/slam/switch_map_hook.sh"))
TASK_SCRIPT = os.environ.get("TASK_SCRIPT", os.path.join(NAV_ROOT, "lite_cog/pipeline/src/pipeline_tracking/scripts/Task.py"))
TASK_DATA = os.environ.get("TASK_DATA_DIR", os.path.join(NAV_ROOT, "lite_cog/pipeline/src/pipeline_tracking/data"))
NAV_SCRIPT = os.environ.get("NAV_START_SCRIPT", os.path.join(NAV_ROOT, "lite_cog/system/scripts/nav/start_nav.sh"))
RVIZ_CFG = os.environ.get("RVIZ_CONFIG", os.path.join(NAV_ROOT, "lite_cog/nav/src/hdl_localization/rviz/hdl_localization.rviz"))
TF_MAP_FRAME = os.environ.get("TF_MAP_FRAME", "map")
TF_BODY_FRAME = os.environ.get("TF_BODY_FRAME", "base_link")

# topic: AMQP 用 . 分隔 (nav.robot-001.cmd)，MQTT 用 / 分隔 (nav/robot-001/cmd)
_SEP = "/" if MQ_TYPE == "mqtt" else "."
CMD_TOPIC = f"nav{_SEP}{MQ_CLIENT_ID}{_SEP}cmd"

# ====== TEB goal tolerance（从配置文件读取，用于到达判断） ======
# 默认值（会在 main() 中从 teb_local_planner_params.yaml 重新加载）
XY_GOAL_TOLERANCE = 0.5
YAW_GOAL_TOLERANCE = 0.6

def _load_teb_tolerances():
    """从 teb_local_planner_params.yaml 读取 xy_goal_tolerance / yaw_goal_tolerance"""
    global XY_GOAL_TOLERANCE, YAW_GOAL_TOLERANCE
    teb_cfg = os.environ.get("TEB_CONFIG",
        os.path.join(NAV_ROOT, "lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml"))
    try:
        with open(teb_cfg, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        teb = cfg.get("TebLocalPlannerROS", cfg)
        XY_GOAL_TOLERANCE = float(teb.get("xy_goal_tolerance", XY_GOAL_TOLERANCE))
        YAW_GOAL_TOLERANCE = float(teb.get("yaw_goal_tolerance", YAW_GOAL_TOLERANCE))
        msg = f"[MQ] TEB goal tolerance loaded: xy={XY_GOAL_TOLERANCE}, yaw={YAW_GOAL_TOLERANCE}"
        print(msg)
        rospy.loginfo(msg)
    except Exception as e:
        msg = f"[MQ] Failed to load TEB config ({e}), using defaults xy={XY_GOAL_TOLERANCE}, yaw={YAW_GOAL_TOLERANCE}"
        print(msg)
        rospy.logwarn(msg)

# ====== MQ 协议抽象 ======
if MQ_TYPE == "mqtt":
    import paho.mqtt.client as mqtt_lib

    _mq_client = mqtt_lib.Client(client_id=MQ_CLIENT_ID, clean_session=True, protocol=mqtt_lib.MQTTv311)
    _mq_client.username_pw_set(MQ_USER, MQ_PASS)
    _mq_pub_lock = threading.Lock()
    _mq_msg_queue = []  # 消费线程把消息放到这里
    _mq_msg_lock = threading.Lock()

    def mq_connect():
        _mq_client.connect(MQ_HOST, MQ_PORT, keepalive=30)
        _mq_client.loop_start()
        rospy.loginfo(f"[MQ] MQTT connected {MQ_HOST}:{MQ_PORT}")

    def mq_publish(topic_suffix, body):
        routing = f"nav/{MQ_CLIENT_ID}/{topic_suffix}"
        try:
            with _mq_pub_lock:
                _mq_client.publish(routing, json.dumps(body, ensure_ascii=False), qos=int(body.get("_qos", 1)))
        except Exception as e:
            rospy.logerr(f"[MQ] publish {routing}: {e}")

    def mq_subscribe(topic_suffix, qos=1):
        topic = f"nav/{MQ_CLIENT_ID}/{topic_suffix}"
        _mq_client.subscribe(topic, qos=qos)

    def mq_get_messages():
        with _mq_msg_lock:
            msgs = list(_mq_msg_queue)
            _mq_msg_queue.clear()
        return msgs

    def _on_mq_message(client, userdata, msg):
        with _mq_msg_lock:
            _mq_msg_queue.append(msg)

    def _on_mq_connect(client, userdata, flags, rc):
        """每次（重）连接时重新订阅，防止 clean_session 导致订阅丢失"""
        if rc == 0:
            topic = f"nav/{MQ_CLIENT_ID}/cmd"
            client.subscribe(topic, qos=2)
            rospy.loginfo(f"[MQ] MQTT re-subscribed {topic}")
        else:
            rospy.logwarn(f"[MQ] MQTT connect failed rc={rc}")

    _mq_client.on_connect = _on_mq_connect
    _mq_client.on_message = _on_mq_message

else:
    import pika as _pika_mod

    _mq_conn = None
    _mq_ch_pub = None

    def mq_connect():
        global _mq_conn, _mq_ch_pub
        params = _pika_mod.ConnectionParameters(
            host=MQ_HOST, port=MQ_PORT, virtual_host=MQ_VHOST,
            credentials=_pika_mod.PlainCredentials(MQ_USER, MQ_PASS), heartbeat=30)
        _mq_conn = _pika_mod.BlockingConnection(params)
        _mq_ch_pub = _mq_conn.channel()
        _mq_ch_pub.exchange_declare(exchange=MQ_EXCHANGE, exchange_type="topic", durable=True)
        rospy.loginfo(f"[MQ] AMQP connected {MQ_HOST}:{MQ_PORT}")

    _pub_lock = threading.Lock()
    def mq_publish(topic_suffix, body):
        routing = f"nav.{MQ_CLIENT_ID}.{topic_suffix}"
        with _pub_lock:
            try:
                if _mq_ch_pub and not _mq_ch_pub.is_closed:
                    _mq_ch_pub.basic_publish(exchange=MQ_EXCHANGE, routing_key=routing,
                                             body=json.dumps(body, ensure_ascii=False))
            except Exception as e:
                rospy.logerr(f"[MQ] publish {routing}: {e}")

    def mq_subscribe(topic_suffix, qos=1):
        pass  # AMQP consumer 单独处理

    def mq_get_messages():
        return []  # AMQP 消费不用队列

# ====== 工具 ======
def _hdr(msg_type):
    return {"msg_type": msg_type}

def _ack(ref, cmd, result, reason=""):
    mq_publish("cmd.ack", {"header": _hdr("cmd_ack"),
        "body": {"ref_msg_id": ref, "cmd": cmd, "result": result, "reason": reason}})

def get_map_list():
    if not os.path.exists(DB_PATH): return []
    try:
        conn = sqlite3.connect(DB_PATH); conn.row_factory = sqlite3.Row
        rows = conn.execute("SELECT id,name,yaml_path,pcd_path,created_at,active FROM maps ORDER BY created_at DESC").fetchall()
        conn.close()
        return [{"id": r["id"], "name": r["name"], "yaml_path": r["yaml_path"],
                 "pcd_path": r["pcd_path"] or "", "created_at": r["created_at"],
                 "active": bool(r["active"])} for r in rows]
    except Exception as e:
        rospy.logerr(f"[MQ] DB: {e}"); return []

# ====== 状态 ======
class NavState:
    def __init__(self):
        self.lock = threading.Lock()
        self.current_map = ""; self.last_goal = None; self.task_proc = None
    def to_dict(self):
        with self.lock:
            return {"current_map": self.current_map}
state = NavState()

# ====== move_base ======
class MBClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.ok = False
    def connect(self, t=10): self.ok = self.client.wait_for_server(rospy.Duration(t)); return self.ok
    def send(self, x, y, yaw, frame="map"):
        g = MoveBaseGoal()
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.header.frame_id = frame
        g.target_pose.pose.position.x = x; g.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        g.target_pose.pose.orientation.x = q[0]; g.target_pose.pose.orientation.y = q[1]
        g.target_pose.pose.orientation.z = q[2]; g.target_pose.pose.orientation.w = q[3]
        self.client.send_goal(g)
    def cancel(self): self.client.cancel_all_goals()
    def state(self): return self.client.get_state()
mb = MBClient()
init_pub = None

def _kill_task():
    with state.lock: p = state.task_proc
    if p and p.poll() is None:
        p.send_signal(signal.SIGINT)
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired: p.kill()
    with state.lock: state.task_proc = None

# ====== 指令处理 ======
def h_map_list(body, ref):
    maps = get_map_list()
    mq_publish("map_list", {"header": _hdr("map_list"), "body": {"maps": maps, "total": len(maps)}})
    _ack(ref, "map_list", "accepted")

def _switch_and_init(mid):
    """切换地图并发布 initialpose 到原点"""
    maps = get_map_list(); t = None
    for m in maps:
        if m["id"] == mid or m["name"] == mid: t = m; break
    if t:
        try:
            rospy.loginfo(f"[MQ] switching map: {t['name']} ({t['id'][:8]}...)")
            subprocess.run(["bash", SWITCH_HOOK, t["yaml_path"], t.get("pcd_path",""), t["name"]],
                           capture_output=True, timeout=30)
            with state.lock: state.current_map = t["name"]
            ip = PoseWithCovarianceStamped()
            ip.header.stamp = rospy.Time.now(); ip.header.frame_id = "map"
            ip.pose.pose.orientation.w = 1.0
            if init_pub: init_pub.publish(ip)
        except: pass
    else:
        rospy.logwarn(f"[MQ] map not found: {mid}")

def _ensure_nav_stack():
    """确保导航栈（hdl_localization + rviz）已启动"""
    # 如果 move_base 未就绪，启动整个导航栈
    if not mb.ok:
        rospy.loginfo("[MQ] nav stack not ready, launching...")
        try:
            subprocess.Popen(["bash", NAV_SCRIPT],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True)
            for _ in range(60):
                if mb.connect(): break
                time.sleep(0.5)
        except Exception as e:
            rospy.logwarn(f"[MQ] nav stack launch failed: {e}")
    # rviz disabled: web UI replaces visualization
    # try:
    #     if subprocess.run(["pgrep", "-f", "rviz"], capture_output=True).returncode != 0:
    #         rospy.loginfo("[MQ] launching rviz...")
    #         subprocess.Popen(["bash", "-c",
    #             f"source /opt/ros/noetic/setup.bash && rviz -d {RVIZ_CFG}"],
    #             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    #             start_new_session=True)
    # except Exception as e:
    #     rospy.logwarn(f"[MQ] rviz launch failed: {e}")

def h_switch_map(body, ref):
    """独立地图切换 — 不导航，只切图 + 发 initialpose + 推送 map_list"""
    mid = body.get("map_id", "") or body.get("map_name", "")
    if not mid:
        _ack(ref, "switch_map", "rejected", "need map_id or map_name")
        return
    _switch_and_init(mid)
    # 推送更新后的 map_list
    maps = get_map_list()
    mq_publish("map_list", {"header": _hdr("map_list"), "body": {"maps": maps, "total": len(maps)}})
    _ack(ref, "switch_map", "accepted")
    rospy.loginfo(f"[MQ] switch_map: {mid}")

def h_nav_single(body, ref):
    g = body.get("goal", {})
    x, y, yaw = g.get("x"), g.get("y"), g.get("yaw")
    if x is None: _ack(ref, "nav_single", "rejected", "need x/y/yaw"); return
    mid = body.get("map_id", "")
    if mid: _switch_and_init(mid)
    # 启动雷达 + 运控（等价于按下 Web 端按钮），gateway 已做幂等保护
    _ensure_lidar_and_motion()
    # ★ 先重置到达状态 + 记录新目标，再启导航栈。
    #    否则 gateway 变 running 后 pub loop 进入 _compute_motion_state()，
    #    旧 _has_arrived=True 会被无条件冻结返回"到达"（最多 60s 直到下面的 reset 执行）。
    with state.lock: state.last_goal = body
    pub.reset_pose_history()
    # 标记"有新目标"，使 Publisher 在 gateway 返回 running 之前就进入 _compute_motion_state
    pub._resumed_after_pause = True
    # gateway 启导航栈 + 通过 gateway /api/nav/simple-goal 发 goal（和 web 2D Nav Goal 完全一致）
    # 始终用 "map" 坐标系，和 web 前端一致（camera_init 没有到 map 的 TF，move_base 会丢弃 goal）
    _gateway_nav_cmd("nav-only")
    # 輪詢等待 move_base action server 就緒（最多等 60 秒）
    rospy.loginfo("[MQ] waiting for move_base action server...")
    ready = False
    for i in range(60):
        if mb.connect(t=1):
            rospy.loginfo(f"[MQ] move_base ready (waited {i+1}s)")
            ready = True
            break
        rospy.sleep(1.0)
    if not ready:
        rospy.logwarn("[MQ] move_base not ready after 60s, sending goal anyway")

    ok, msg = _call_gateway("/api/nav/simple-goal", {"x": float(x), "y": float(y), "yaw": float(yaw), "frame_id": "map"})
    rospy.loginfo(f"[MQ] simple-goal via gateway: ok={ok} x={x} y={y} yaw={yaw}")
    mq_publish("nav_points", {"header": _hdr("nav_points"),
        "body": {"points": [{"id": "goal", "x": float(x), "y": float(y), "yaw": float(yaw)}]}})
    _ack(ref, "nav_single", "accepted")

def h_nav_goal(body, ref):
    """单独发送单点导航目标 — 直接发给 move_base，不切换地图、不走 gateway 编排"""
    g = body.get("goal", {})
    x, y, yaw = g.get("x"), g.get("y"), g.get("yaw")
    if x is None or y is None or yaw is None:
        _ack(ref, "nav_goal", "rejected", "need goal.x, goal.y, goal.yaw")
        return
    # 直接发给 move_base（不切换地图、不等 nav stack）
    frame = g.get("frame_id", "map")
    mb.send(float(x), float(y), float(yaw), frame=str(frame))
    # 通知 gateway 更新 Web 端目标点显示（broadcast nav_goal 事件）
    ok, _ = _call_gateway("/api/nav/simple-goal", {"x": float(x), "y": float(y), "yaw": float(yaw), "frame_id": str(frame)})
    rospy.loginfo(f"[MQ] nav_goal: x={x}, y={y}, yaw={yaw}, frame={frame}, web_broadcast={ok}")
    mq_publish("nav_points", {"header": _hdr("nav_points"),
        "body": {"points": [{"id": "goal", "x": float(x), "y": float(y), "yaw": float(yaw)}]}})
    with state.lock: state.last_goal = body
    pub.reset_pose_history()
    # 标记"暂停后恢复"，使 Publisher 在 gateway 状态仍为 paused 时也计算实际运动状态
    pub._resumed_after_pause = True
    _ack(ref, "nav_goal", "accepted")

def h_nav_multi(body, ref):
    wps = body.get("waypoints", [])
    if not wps: _ack(ref, "nav_multi", "rejected", "need waypoints"); return
    mid = body.get("map_id", "")
    if mid: _switch_and_init(mid)
    # 启动雷达 + 运控（等价于按下 Web 端按钮），gateway 已做幂等保护
    _ensure_lidar_and_motion()
    os.makedirs(TASK_DATA, exist_ok=True)
    for f in os.listdir(TASK_DATA):
        if f.endswith(".json"): os.unlink(os.path.join(TASK_DATA, f))
    for i, wp in enumerate(wps):
        y = float(wp.get("yaw", 0))
        rec = {"order": i, "robot_pose": {"pos_x": float(wp["x"]), "pos_y": float(wp["y"]), "pos_z": 0,
                "ori_x": 0, "ori_y": 0, "ori_z": math.sin(y/2), "ori_w": math.cos(y/2)},
               "option": {"even_low_speed": False, "even_medium_speed": False, "uneven_high_step": False}}
        with open(os.path.join(TASK_DATA, f"{i}.json"), "w") as fp: json.dump(rec, fp)
    # 发布途经点到 MQ，供 Web 端显示
    nav_pts = [{"id": wp.get("id", f"wp-{i}"), "x": float(wp["x"]), "y": float(wp["y"]),
                "yaw": float(wp.get("yaw", 0))} for i, wp in enumerate(wps)]
    mq_publish("nav_points", {"header": _hdr("nav_points"), "body": {"points": nav_pts}})
    # gateway 启导航栈 + Task.py，设状态为 running
    pub.reset_pose_history()
    _gateway_nav_cmd("start")
    _ack(ref, "nav_multi", "accepted")

def h_nav_pause(body, ref):
    mb.cancel()
    _gateway_nav_cmd("pause")
    pub._resumed_after_pause = False  # 清除暂停恢复标记，正确报告"已暂停"
    _ack(ref, "nav_pause", "accepted")

def h_nav_cancel(body, ref):
    global mb
    mb.cancel()
    _gateway_nav_cmd("stop")
    state.last_goal = None
    mb = MBClient()
    pub._resumed_after_pause = False  # 清除暂停恢复标记
    _ack(ref, "nav_cancel", "accepted")

def h_nav_resume(body, ref):
    if state.last_goal:
        _gateway_nav_cmd("resume")
        h_nav_single(state.last_goal, ref)
    else: _ack(ref, "nav_resume", "rejected", "no previous goal")

def h_relocalize(body, ref):
    pose = body.get("pose", {})
    x, y, yaw = pose.get("x"), pose.get("y"), pose.get("yaw")
    if x is None or y is None or yaw is None:
        _ack(ref, "relocalize", "rejected", "need pose.x, pose.y, pose.yaw")
        return
    frame = body.get("frame_id", "map")
    if init_pub:
        ip = PoseWithCovarianceStamped()
        ip.header.stamp = rospy.Time.now()
        ip.header.frame_id = str(frame)
        ip.pose.pose.position.x = float(x)
        ip.pose.pose.position.y = float(y)
        q = tf.transformations.quaternion_from_euler(0, 0, float(yaw))
        ip.pose.pose.orientation.x = q[0]
        ip.pose.pose.orientation.y = q[1]
        ip.pose.pose.orientation.z = q[2]
        ip.pose.pose.orientation.w = q[3]
        init_pub.publish(ip)
    _ack(ref, "relocalize", "accepted")
    rospy.loginfo(f"[MQ] relocalize: x={x}, y={y}, yaw={yaw}, frame={frame}")

# ====== 运控指令 ======
GATEWAY_URL = os.environ.get("GATEWAY_URL", "http://127.0.0.1:8080")
GATEWAY_USER = os.environ.get("GATEWAY_USER", "admin")
GATEWAY_PASS = os.environ.get("GATEWAY_PASS", "admin123")
_gateway_token = None
cmd_vel_pub = None
sport_cmd_pub = None

def _gateway_login():
    global _gateway_token
    try:
        data = json.dumps({"username": GATEWAY_USER, "password": GATEWAY_PASS}).encode()
        req = urllib.request.Request(f"{GATEWAY_URL}/api/auth/login", data=data,
                                      headers={"Content-Type": "application/json"}, method="POST")
        resp = json.loads(urllib.request.urlopen(req, timeout=5).read())
        _gateway_token = resp.get("token", "")
        return bool(_gateway_token)
    except Exception as e:
        rospy.logwarn(f"[MQ] gateway login failed: {e}")
        return False

def _call_gateway(endpoint, data=None, method="POST"):
    global _gateway_token
    if not _gateway_token and not _gateway_login():
        return False, "gateway auth failed"
    url = f"{GATEWAY_URL}{endpoint}"
    body = json.dumps(data or {}).encode() if data else None
    headers = {"Content-Type": "application/json", "Authorization": f"Bearer {_gateway_token}"}
    req = urllib.request.Request(url, data=body, headers=headers, method=method)
    try:
        resp = urllib.request.urlopen(req, timeout=5)
        if resp.status == 200: return True, resp.read().decode()
        elif resp.status == 401:
            if _gateway_login():
                headers["Authorization"] = f"Bearer {_gateway_token}"
                req2 = urllib.request.Request(url, data=body, headers=headers, method=method)
                resp2 = urllib.request.urlopen(req2, timeout=5)
                return resp2.status == 200, resp2.read().decode()
        return False, f"HTTP {resp.status}"
    except urllib.error.HTTPError as e:
        return False, f"HTTP {e.code}: {e.reason}"
    except Exception as e:
        return False, str(e)

def _gateway_nav_cmd(command):
    """调用 gateway /api/nav/command，gateway 是唯一状态源"""
    ok, msg = _call_gateway("/api/nav/command", {"command": command})
    if ok: rospy.loginfo(f"[MQ] gateway nav {command} OK")
    else: rospy.logwarn(f"[MQ] gateway nav {command} failed: {msg}")
    return ok

def _ensure_lidar_and_motion():
    """启动导航时同时启动雷达和运控（等价于按下 Web 端按钮）"""
    ok_l, _ = _call_gateway("/api/lidar/start", method="POST")
    ok_m, _ = _call_gateway("/api/motion/start", method="POST")
    if ok_l: rospy.loginfo("[MQ] lidar start → gateway OK")
    if ok_m: rospy.loginfo("[MQ] motion start → gateway OK")
    return ok_l and ok_m

def _ensure_motion_pubs():
    global cmd_vel_pub, sport_cmd_pub
    if cmd_vel_pub is None:
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    if sport_cmd_pub is None:
        sport_cmd_pub = rospy.Publisher("/go2/sport_cmd", RosString, queue_size=5)

def h_motor_start(body, ref):
    ok, msg = _call_gateway("/api/motion/start")
    if ok: _ack(ref, "motor_start", "accepted"); rospy.loginfo("[MQ] motor_start → gateway OK")
    else: _ack(ref, "motor_start", "rejected", msg[:200])

def h_motor_stop(body, ref):
    ok, msg = _call_gateway("/api/motion/stop")
    if ok: _ack(ref, "motor_stop", "accepted"); rospy.loginfo("[MQ] motor_stop → gateway OK")
    else: _ack(ref, "motor_stop", "rejected", msg[:200])

def h_motor_stand(body, ref):
    _ensure_motion_pubs()
    if sport_cmd_pub:
        sport_cmd_pub.publish(RosString(data="stand_up")); _ack(ref, "motor_stand", "accepted")
    else: _ack(ref, "motor_stand", "rejected", "publisher not ready")

def h_motor_sit(body, ref):
    _ensure_motion_pubs()
    if sport_cmd_pub:
        sport_cmd_pub.publish(RosString(data="sit")); _ack(ref, "motor_sit", "accepted")
    else: _ack(ref, "motor_sit", "rejected", "publisher not ready")

def h_motor_damp(body, ref):
    _ensure_motion_pubs()
    if sport_cmd_pub:
        sport_cmd_pub.publish(RosString(data="damp")); _ack(ref, "motor_damp", "accepted")
    else: _ack(ref, "motor_damp", "rejected", "publisher not ready")

def h_motor_move(body, ref):
    vx = float(body.get("vx", 0)); vy = float(body.get("vy", 0)); vyaw = float(body.get("vyaw", 0))
    _ensure_motion_pubs()
    if cmd_vel_pub:
        tw = Twist(); tw.linear.x = vx; tw.linear.y = vy; tw.angular.z = vyaw
        cmd_vel_pub.publish(tw); _ack(ref, "motor_move", "accepted")
    else: _ack(ref, "motor_move", "rejected", "publisher not ready")

HANDLERS = {"map_list": h_map_list, "nav_single": h_nav_single,
            "nav_goal": h_nav_goal, "switch_map": h_switch_map,
            "nav_multi": h_nav_multi, "nav_pause": h_nav_pause, "nav_resume": h_nav_resume,
            "nav_cancel": h_nav_cancel, "relocalize": h_relocalize,
            "motor_start": h_motor_start, "motor_stop": h_motor_stop,
            "motor_stand": h_motor_stand, "motor_sit": h_motor_sit,
            "motor_damp": h_motor_damp, "motor_move": h_motor_move}

def process_cmd(payload_bytes):
    """处理收到的指令（AMQP/MQTT 共用）"""
    body_str = payload_bytes.decode("utf-8") if isinstance(payload_bytes, bytes) else payload_bytes
    body_str = body_str.strip().replace("\n", "").replace("\r", "").replace("\t", " ")
    try: msg = json.loads(body_str)
    except json.JSONDecodeError as e:
        rospy.logwarn(f"[MQ] bad JSON: {e} | raw={str(body_str)[:200]}")
        return
    h, p = msg.get("header", {}), msg.get("body", {})
    cmd, ref = p.get("cmd", ""), h.get("msg_id", "")
    rospy.loginfo(f"[MQ] recv cmd: {cmd}")
    f = HANDLERS.get(cmd)
    if f:
        try: f(p, ref)
        except Exception as e: _ack(ref, cmd, "rejected", str(e))
    else: _ack(ref, cmd, "rejected", f"unknown: {cmd}")

# ====== 发布者 ======
class Publisher:
    def __init__(self):
        self.tf_listener = None
        self.last_plan = None
        self.last_local_plan = None
        self._run = False
        self.sub = None
        self.sub_local_plan = None
        self.sub_cmd_vel = None
        self.sub_simple_goal = None
        self._last_simple_goal = None  # (x, y, yaw) 最近一次 /move_base_simple/goal，web 端真实目标点
        self.last_cmd_vel = None
        self.last_cmd_vel_time = 0.0
        self._pose_history = []        # [(t, x, y, yaw), ...] 最近位置历史
        self._start_pos = None         # (x, y) 收到目标时的位置，用于判断是否已起步
        self._start_yaw = None         # 收到目标时的朝向(rad)，用于判断原地旋转起步
        self._goal_received_at = 0.0   # 收到目标的时间戳
        self._has_departed = False     # 是否已经离开起点足够远（含原地旋转）
        self._computed_nav_state = "idle"
        self._has_arrived = False      # 到达目标后持久化，防止 plan 清空后状态被覆盖
        self._last_arrived_goal_pos = None  # (x, y) 上次到达的目标点位置，用于防止旧路径误判
        self._blocked_since = 0.0      # 阻塞条件首次满足的时间戳，0=未满足，用于 10s 持续判断
        self._resumed_after_pause = False  # 暂停后通过 MQ nav_goal 恢复了运动
        self._last_pub_pos = None      # (x, y) 上次发布的坐标，用于变化阈值判断
        self._last_pub_yaw = None      # 上次发布的 yaw 角度
        self._last_pub_status = None   # 上次发布的 nav_state，避免重复发布相同状态
        self._last_route_hash = None    # 上次发布的 route 哈希，避免重复发布相同路径
        self._last_local_route_hash = None  # 上次发布的 local_route 哈希
    def _init(self):
        self.tf_listener = tf.TransformListener()
        self.sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self._on_plan)
        self.sub_local_plan = rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self._on_local_plan)
        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self._on_cmd_vel)
        self.sub_simple_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._on_simple_goal)
    def _publish(self, topic_suffix, body):
        """线程安全发布"""
        mq_publish(topic_suffix, body)
    def _on_plan(self, m): self.last_plan = m
    def _on_local_plan(self, m): self.last_local_plan = m
    def _on_cmd_vel(self, m):
        self.last_cmd_vel = m
        self.last_cmd_vel_time = time.time()
    def _on_simple_goal(self, m):
        """记录 /move_base_simple/goal — web 端发送的真实目标点"""
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w])
        self._last_simple_goal = (m.pose.position.x, m.pose.position.y, yaw)
        # 有新的 simple_goal 意味着新导航开始，重置到达状态
        self._has_arrived = False
        self._last_arrived_goal_pos = None
        self._goal_received_at = time.time()
        self._start_pos = None
        self._has_departed = False
        self._blocked_since = 0.0
    def reset_pose_history(self):
        """收到新目标时清空位姿历史，重置起步状态"""
        self._pose_history.clear()
        self._goal_received_at = time.time()
        self._start_pos = None       # 下个循环从 TF 获取当前位置作为起点
        self._start_yaw = None       # 下个循环从 TF 获取当前朝向作为起点
        self._has_departed = False
        self._has_arrived = False    # 新目标 → 重置到达状态
        self._blocked_since = 0.0    # 新目标 → 重置阻塞计时
        self.last_plan = None         # 清除旧规划路径
        self.last_local_plan = None  # 清除旧局部路径
        self._last_local_route_hash = None
        # 注意：不清除 _last_arrived_goal_pos，它用于防止旧路径重入时误判到达
        self._last_pub_status = None  # 强制重新发布状态
    def _is_stationary(self):
        """检查机器人是否静止（cmd_vel 为主 + 位姿历史为辅）

        cmd_vel 是控制器直接指令，到达目标后 TEB 会发零速度，是最可靠的静止信号。
        位姿历史作为 fallback（1Hz 采样，窗口需 ≥2 秒才能抓到足够样本）。
        """
        now = time.time()
        STATIONARY_LINEAR = 0.05     # 线速度低于此值视为静止 (m/s)
        STATIONARY_ANGULAR = 0.05    # 角速度低于此值视为静止 (rad/s)

        # 方法1: cmd_vel 命令速度为零 → 控制器认为已到达，最直接可信
        if self.last_cmd_vel is not None:
            cmd_age = now - self.last_cmd_vel_time
            if cmd_age < 2.0:  # cmd_vel 最近 2 秒内有更新
                v_linear = abs(self.last_cmd_vel.linear.x)
                v_angular = abs(self.last_cmd_vel.angular.z)
                if v_linear < STATIONARY_LINEAR and v_angular < STATIONARY_ANGULAR:
                    return True

        # 方法2: 位姿历史计算实际速度 (fallback, 配合 1Hz run loop 用 2 秒窗口)
        STATIONARY_WINDOW = 2.0
        recent = [p for p in self._pose_history if now - p[0] <= STATIONARY_WINDOW]
        if len(recent) >= 2:
            first = recent[0]
            last = recent[-1]
            dt = last[0] - first[0]
            if dt >= 0.5:
                dx = last[1] - first[1]
                dy = last[2] - first[2]
                linear_vel = math.sqrt(dx*dx + dy*dy) / dt
                dyaw = abs(last[3] - first[3])
                dyaw = min(dyaw, 2 * math.pi - dyaw)
                angular_vel = dyaw / dt
                if linear_vel < STATIONARY_LINEAR and angular_vel < STATIONARY_ANGULAR:
                    return True

        return False

    def _compute_motion_state(self):
        """计算详细运动状态：运动中/阻塞/到达

        20cm 直连距离分界线：
        - dist_to_goal < 0.2m → 进入到达判定，跳过阻塞判定
        - dist_to_goal >= 0.2m → 进入阻塞判定，跳过到达判定
        """
        has_plan = self.last_plan is not None and len(self.last_plan.poses) > 0

        # 脏路径检测：旧 move_base 在被 kill 前最后发的一条 plan，
        # 终点 = _last_arrived_goal_pos（上一轮已到达的位置），机器人就在那，
        # 导致 near_goal=True → 阻塞判定被整体跳过。
        if has_plan and self._last_arrived_goal_pos is not None and self._goal_received_at > 0:
            try:
                pgx = self.last_plan.poses[-1].pose.position.x
                pgy = self.last_plan.poses[-1].pose.position.y
                d = math.sqrt((pgx - self._last_arrived_goal_pos[0])**2 +
                              (pgy - self._last_arrived_goal_pos[1])**2)
                if d < XY_GOAL_TOLERANCE:
                    # 脏路径：指向已到达的旧目标，视为无路线
                    self.last_plan = None
                    self.last_local_plan = None
                    has_plan = False
            except: pass

        now = time.time()
        ARRIVAL_GATE = 0.2  # 20cm：到达/阻塞互斥分界线

        # 0) 计算当前位置到目标点的直连距离（TF 只查一次，后续复用）
        cur_pos = None       # (x, y)
        cur_ori = None       # quaternion [x,y,z,w]
        dist_to_goal = None
        goal_pt = None       # (x, y) 目标点坐标
        if has_plan:
            goal_pt = (self.last_plan.poses[-1].pose.position.x,
                       self.last_plan.poses[-1].pose.position.y)
        elif state.last_goal:
            g = state.last_goal.get("goal", {})
            if "x" in g and "y" in g:
                goal_pt = (float(g["x"]), float(g["y"]))
        if goal_pt is not None:
            try:
                pos, ori = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                cur_pos = (pos[0], pos[1])
                cur_ori = ori
                dist_to_goal = math.sqrt((cur_pos[0] - goal_pt[0])**2 + (cur_pos[1] - goal_pt[1])**2)
            except: pass
        near_goal = dist_to_goal is not None and dist_to_goal < ARRIVAL_GATE

        # === 到达判定：只在 20cm 内才可能，已到达则跳过防止状态回退 ===
        if has_plan and near_goal and not self._has_arrived:
            if self._goal_received_at == 0.0:
                self._goal_received_at = now
            try:
                # 计算目标点位置用于去重
                is_same_goal = (
                    self._last_arrived_goal_pos is not None and
                    math.sqrt((goal_pt[0] - self._last_arrived_goal_pos[0])**2 +
                              (goal_pt[1] - self._last_arrived_goal_pos[1])**2) < XY_GOAL_TOLERANCE
                )
                # 同一目标 + 未标记到达 = 收到新目标后旧路径重入，不触发到达
                if is_same_goal and not self._has_arrived:
                    pass  # fall through 到运动检测
                else:
                    _, _, cur_yaw = tf.transformations.euler_from_quaternion(
                        [cur_ori[0], cur_ori[1], cur_ori[2], cur_ori[3]])
                    target_yaw = None
                    last_ori = self.last_plan.poses[-1].pose.orientation
                    if not (abs(last_ori.x) < 1e-6 and abs(last_ori.y) < 1e-6 and
                            abs(last_ori.z) < 1e-6 and abs(last_ori.w) < 1e-6):
                        _, _, target_yaw = tf.transformations.euler_from_quaternion(
                            [last_ori.x, last_ori.y, last_ori.z, last_ori.w])
                    if target_yaw is None:
                        lg = state.last_goal
                        if lg:
                            g = lg.get("goal", {})
                            if "yaw" in g:
                                target_yaw = float(g["yaw"])
                    if target_yaw is not None:
                        yaw_diff = abs(cur_yaw - target_yaw)
                        yaw_diff = min(yaw_diff, 2 * math.pi - yaw_diff)
                        if yaw_diff < YAW_GOAL_TOLERANCE:
                            if self._is_stationary():
                                if not is_same_goal:
                                    self._last_arrived_goal_pos = goal_pt
                                self._has_arrived = True
                                return "到达"
                            else:
                                return "对齐中"
                        else:
                            return "对齐中"
                    else:
                        if self._is_stationary():
                            if not is_same_goal:
                                self._last_arrived_goal_pos = goal_pt
                            self._has_arrived = True
                            return "到达"
                        else:
                            return "对齐中"
            except: pass

        # 1.5) 到达后状态冻结：防止机器人微小漂移导致状态在"到达/运动中"间振荡。
        #       但如果新目标与上次到达的目标不同（新导航指令），
        #       则清除到达标记，继续走下面的运动/阻塞判定。
        #       直接从 /move_base_simple/goal 读取 web 端真实目标点，不依赖 plan。
        if self._has_arrived and self._last_arrived_goal_pos is not None \
                and self._last_simple_goal is not None:
            try:
                new_goal = (self._last_simple_goal[0], self._last_simple_goal[1])
                same = (math.sqrt((new_goal[0] - self._last_arrived_goal_pos[0])**2 +
                                  (new_goal[1] - self._last_arrived_goal_pos[1])**2)
                        < XY_GOAL_TOLERANCE)
                if not same:
                    self._has_arrived = False
                    self._last_arrived_goal_pos = None
            except: pass
            if self._has_arrived:
                return "到达"

        # 2) 有活跃目标时先记录起点位置与朝向（在阻塞判断前必须完成）
        if self._start_pos is None and self._goal_received_at > 0:
            if cur_pos is not None and cur_ori is not None:
                self._start_pos = cur_pos
                _, _, start_yaw = tf.transformations.euler_from_quaternion(
                    [cur_ori[0], cur_ori[1], cur_ori[2], cur_ori[3]])
                self._start_yaw = start_yaw
            else:
                try:
                    pos, ori = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                    self._start_pos = (pos[0], pos[1])
                    _, _, start_yaw = tf.transformations.euler_from_quaternion([ori[0],ori[1],ori[2],ori[3]])
                    self._start_yaw = start_yaw
                except: pass

        # === 无路线超时（与目标距离无关：目标在障碍物里/定位漂移等） ===
        BLOCK_NO_PLAN_TIMEOUT = 10.0  # 有目标但无路线超过此时长 → 阻塞
        if not has_plan and not self._has_arrived and self._goal_received_at > 0 \
                and now - self._goal_received_at > BLOCK_NO_PLAN_TIMEOUT:
            return "阻塞"

        # 4) 无路线且无活跃目标 → 运动中（等待规划，TF 暂不可用）
        if not has_plan and self._start_pos is None:
            return "运动中"

        # === 阻塞判定：只在 20cm 外才可能（运动速度/位移检测） ===
        if not near_goal:
            DEPART_DIST = 0.5        # 离开起点超过此距离才算已起步 (m)
            BLOCK_WINDOW = 3.0       # 阻塞判断窗口（秒）
            BLOCK_SPEED = 0.03       # 平均线速度低于此值视为卡死 (m/s)
            GOAL_TIMEOUT = 5.0       # 收到目标后超时未起步也算阻塞 (s)
            BLOCK_PERSIST = 10.0     # 阻塞条件需持续 10 秒才上报
            is_blocked = False       # 本轮是否满足阻塞条件
            try:
                if cur_pos is None or cur_ori is None:
                    pos, ori = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                    cur_x, cur_y = pos[0], pos[1]
                    _, _, yaw = tf.transformations.euler_from_quaternion([ori[0],ori[1],ori[2],ori[3]])
                else:
                    cur_x, cur_y = cur_pos
                    _, _, yaw = tf.transformations.euler_from_quaternion(
                        [cur_ori[0], cur_ori[1], cur_ori[2], cur_ori[3]])

                # 首次记录起点（step 2 中 TF 查询失败时的 fallback）
                if self._start_pos is None:
                    self._start_pos = (cur_x, cur_y)
                    self._start_yaw = yaw

                # 判断是否已离开起点（平移 + 原地旋转都算"起步"）
                if not self._has_departed:
                    dist_from_start = math.sqrt((cur_x - self._start_pos[0])**2 + (cur_y - self._start_pos[1])**2)
                    yaw_from_start = 0.0
                    if self._start_yaw is not None:
                        yaw_from_start = abs(yaw - self._start_yaw)
                        yaw_from_start = min(yaw_from_start, 2 * math.pi - yaw_from_start)
                    YAW_DEPART_THRESH = 0.3  # 旋转超过 ~17° 视为已起步
                    if dist_from_start >= DEPART_DIST or yaw_from_start >= YAW_DEPART_THRESH:
                        self._has_departed = True
                        self._pose_history.clear()  # 清空起步前静止采样，阻塞判定只看起步后的运动
                    elif self._goal_received_at > 0 and now - self._goal_received_at > 10.0:
                        return "阻塞"  # 10秒未起步 → 直接阻塞，不等待 BLOCK_PERSIST
                    # else: 未起步未超时 → 不阻塞，is_blocked 保持 False

                else:
                    # 已起步 → 检查是否在足够长的时间内位移过小
                    if len(self._pose_history) >= 5:
                        first = self._pose_history[0]
                        last_ph = self._pose_history[-1]
                        dt_hist = last_ph[0] - first[0]
                        if dt_hist >= BLOCK_WINDOW:
                            dx = last_ph[1] - first[1]
                            dy = last_ph[2] - first[2]
                            avg_speed = math.sqrt(dx*dx + dy*dy) / dt_hist
                            if avg_speed < BLOCK_SPEED:
                                is_blocked = True
                            else:
                                # 原地旋转检查：几乎不移动但角速度明显 → 卡住原地转
                                dyaw = abs(last_ph[3] - first[3])
                                dyaw = min(dyaw, 2 * math.pi - dyaw)
                                avg_angular = dyaw / dt_hist
                                if avg_speed < 0.05 and avg_angular > 0.2:
                                    is_blocked = True
            except: pass

            # 阻塞条件需持续 BLOCK_PERSIST 秒才上报
            if is_blocked:
                if self._blocked_since == 0.0:
                    self._blocked_since = now
                elif now - self._blocked_since >= BLOCK_PERSIST:
                    return "阻塞"
                # 未满 10 秒 → 继续返回运动中，累积计时
            else:
                self._blocked_since = 0.0  # 条件不满足 → 重置计时

        return "运动中"
    def run(self):
        self._init()
        self._run = True
        self._goal_received_at = 0.0   # 0 表示未收到目标，阻塞判断不生效
        last_hb = 0
        POS_THRESHOLD = 0.05   # 位置变化超过 5cm 才发送
        YAW_THRESHOLD = 0.05   # 角度变化超过 ~2.86° 才发送
        POSE_FORCE_INTERVAL = 5.0  # 即使无变化，每 5 秒强制发送一次
        STATUS_FORCE_INTERVAL = 2.0  # 状态即使无变化，每 2 秒也强制发送一次
        self._last_pub_pos = None
        self._last_pub_yaw = None
        self._last_pose_force = 0.0
        self._last_status_force = 0.0
        while self._run and not rospy.is_shutdown():
            now = time.time()
            try:
                pos, ori = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                roll, pitch, yaw = tf.transformations.euler_from_quaternion([ori[0],ori[1],ori[2],ori[3]])
                # 更新位姿历史（用于阻塞判断）
                self._pose_history.append((now, pos[0], pos[1], yaw))
                self._pose_history = [p for p in self._pose_history if now - p[0] < 5.0]
                # 变化阈值判断：位置或角度变化足够大，或超过强制发送间隔
                should_pub = False
                if self._last_pub_pos is None or self._last_pub_yaw is None:
                    should_pub = True  # 首次发送
                else:
                    dx = pos[0] - self._last_pub_pos[0]
                    dy = pos[1] - self._last_pub_pos[1]
                    dist_moved = math.sqrt(dx*dx + dy*dy)
                    yaw_diff = abs(yaw - self._last_pub_yaw)
                    yaw_diff = min(yaw_diff, 2 * math.pi - yaw_diff)  # 角度环绕
                    if dist_moved >= POS_THRESHOLD or yaw_diff >= YAW_THRESHOLD:
                        should_pub = True
                    elif now - self._last_pose_force >= POSE_FORCE_INTERVAL:
                        should_pub = True  # 超时强制发送
                if should_pub:
                    self._publish("pose", {"header": _hdr("nav_pose"),
                        "body": {"frame_id": TF_MAP_FRAME, "position": {"x":pos[0],"y":pos[1],"z":pos[2]},
                                 "orientation": {"roll": roll, "pitch": pitch, "yaw": yaw},
                                 "localization_quality": "good"}})
                    self._last_pub_pos = (pos[0], pos[1])
                    self._last_pub_yaw = yaw
                    self._last_pose_force = now
            except: pass
            try:
                # 从 gateway 读唯一状态值，不维护本地副本
                ok, data = _call_gateway("/api/snapshot", method="GET")
                gw_nav = "idle"; gw_cmd = ""
                if ok:
                    try:
                        d = json.loads(data)
                        r = d.get("runtime", {})
                        gw_nav = r.get("navStatus", "idle")
                        gw_mode = r.get("navMode", "none")
                        gw_cmd = "nav_multi" if gw_mode == "multi" else ("nav_single" if gw_mode == "single" else "")
                        # 定位状态：体素障碍点云(voxel_grid)是否在更新
                        last_voxel = r.get("lastVoxelAt")
                        if last_voxel:
                            voxel_dt = (datetime.now(timezone.utc) - datetime.fromisoformat(last_voxel.replace("Z", "+00:00"))).total_seconds()
                            if gw_nav == "running" and voxel_dt > 3.0:
                                gw_nav = "loc_lost"  # 导航中但定位丢失
                    except: pass
                self._gw_nav = gw_nav  # 保留原始 gateway 状态
                if gw_nav == "running":
                    computed = self._compute_motion_state()
                elif self._resumed_after_pause:
                    # MQ nav_goal 直接发给 move_base，不改变 gateway 状态。
                    # 无论 gateway 是 paused/idle/stopped，只要 nav_goal 已发送，
                    # 就应该根据实际运动状态计算，而非僵死报告 gateway 状态。
                    computed = self._compute_motion_state()
                else:
                    # gateway 非 running 且无 nav_goal 恢复标记：
                    # - 已到达 → 保持"到达"不变，直到收到新目标
                    # - 未到达 → 跟随 gateway 状态
                    if self._has_arrived:
                        computed = "到达"
                    else:
                        computed = gw_nav
                # gateway 切换到 running 后第一分支接管，清除恢复标记；
                # 其他非 running 状态保留标记，让第二分支继续计算运动状态。
                # 到达后也清除：nav_goal 导航已完成。
                if gw_nav == "running" or computed == "到达":
                    self._resumed_after_pause = False
                self._computed_nav_state = computed
                # 状态变化时立即发布，且每 STATUS_FORCE_INTERVAL 秒强制发送一次
                if computed != self._last_pub_status or now - self._last_status_force >= STATUS_FORCE_INTERVAL:
                    self._publish("status", {"header": _hdr("nav_status"),
                        "body": {"nav_state": computed, "current_cmd": gw_cmd,
                                 "current_cmd_id": "", "current_map": state.current_map}})
                    self._last_pub_status = computed
                    self._last_status_force = now
            except: pass
            try:
                if self.last_plan and self.last_plan.poses:
                    # 只在路径变化时发布 route（比较路径长度+首尾坐标）
                    route_key = (len(self.last_plan.poses),
                                 self.last_plan.poses[0].pose.position.x,
                                 self.last_plan.poses[0].pose.position.y,
                                 self.last_plan.poses[-1].pose.position.x,
                                 self.last_plan.poses[-1].pose.position.y)
                    route_hash = hash(route_key)
                    if route_hash != self._last_route_hash:
                        self._last_route_hash = route_hash
                        pts_raw = [{"x":p.pose.position.x,"y":p.pose.position.y} for p in self.last_plan.poses[:200]]
                        # 计算路径长度
                        path_len = 0.0
                        for i in range(1, len(pts_raw)):
                            dx = pts_raw[i]["x"] - pts_raw[i-1]["x"]
                            dy = pts_raw[i]["y"] - pts_raw[i-1]["y"]
                            path_len += (dx*dx + dy*dy) ** 0.5
                        # source: 当前位置, target: 最后目标点
                        source = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                        target = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                        try:
                            pos, ori = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                            _, _, yaw = tf.transformations.euler_from_quaternion([ori[0],ori[1],ori[2],ori[3]])
                            source = {"x": round(pos[0], 3), "y": round(pos[1], 3), "yaw": round(yaw, 4)}
                        except: pass
                        lg = state.last_goal
                        if lg:
                            g = lg.get("goal", {})
                            target = {"x": float(g.get("x", 0)), "y": float(g.get("y", 0)), "yaw": float(g.get("yaw", 0))}
                        else:
                            # 非 MQ 发起的导航（如 Web 端），fallback 到规划路径实际终点
                            ep = self.last_plan.poses[-1].pose
                            target = {"x": round(ep.position.x, 3), "y": round(ep.position.y, 3), "yaw": 0.0}
                        self._publish("route", {"header": _hdr("nav_route"),
                            "body": {"route_id": str(uuid.uuid4()), "source": source, "target": target,
                                     "path": pts_raw, "path_length": round(path_len, 2)}})
            except: pass
            # === 局部路径 (TEB local plan) ===
            try:
                if self.last_local_plan and self.last_local_plan.poses:
                    # 用 (长度, 首坐标, 尾坐标) 做哈希去重
                    lp = self.last_local_plan
                    local_key = (len(lp.poses),
                                 lp.poses[0].pose.position.x,
                                 lp.poses[0].pose.position.y,
                                 lp.poses[-1].pose.position.x,
                                 lp.poses[-1].pose.position.y)
                    local_hash = hash(local_key)
                    if local_hash != self._last_local_route_hash:
                        self._last_local_route_hash = local_hash
                        pts_raw = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in lp.poses]
                        # 计算局部路径长度
                        path_len = 0.0
                        for i in range(1, len(pts_raw)):
                            dx = pts_raw[i]["x"] - pts_raw[i-1]["x"]
                            dy = pts_raw[i]["y"] - pts_raw[i-1]["y"]
                            path_len += (dx*dx + dy*dy) ** 0.5
                        # source: 当前位置 (和全局路径一致，从 TF 获取)
                        source = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                        try:
                            pos, ori = self.tf_listener.lookupTransform(TF_MAP_FRAME, TF_BODY_FRAME, rospy.Time(0))
                            _, _, yaw = tf.transformations.euler_from_quaternion([ori[0],ori[1],ori[2],ori[3]])
                            source = {"x": round(pos[0], 3), "y": round(pos[1], 3), "yaw": round(yaw, 4)}
                        except: pass
                        # target: 局部路径终点，yaw 从最后一段方向计算
                        target = {"x": round(lp.poses[-1].pose.position.x, 3),
                                  "y": round(lp.poses[-1].pose.position.y, 3), "yaw": 0.0}
                        if len(lp.poses) >= 2:
                            p1 = lp.poses[-2].pose.position
                            p2 = lp.poses[-1].pose.position
                            target["yaw"] = round(math.atan2(p2.y - p1.y, p2.x - p1.x), 4)
                        self._publish("local_route", {"header": _hdr("nav_local_route"),
                            "body": {"route_id": str(uuid.uuid4()), "source": source, "target": target,
                                     "path": pts_raw, "path_length": round(path_len, 2)}})
            except: pass
            if now - last_hb >= 5:
                last_hb = now
                try: self._publish("heartbeat", {"header": _hdr("heartbeat"),
                    "body": {"online":True,"nav_state": getattr(self, '_computed_nav_state', 'idle')}})
                except: pass
            time.sleep(1.0)
    def start(self):
        t = threading.Thread(target=self.run, daemon=True); t.start()
pub = Publisher()

# ====== 消费者 ======
class Consumer:
    def __init__(self):
        self._run = False
    def run(self):
        self._run = True
        while self._run and not rospy.is_shutdown():
            try:
                if MQ_TYPE == "mqtt":
                    self._run_mqtt()
                else:
                    self._run_amqp()
            except Exception as e:
                rospy.logwarn(f"[MQ] consumer: {e}, retry 5s...")
                time.sleep(5)

    def _run_mqtt(self):
        mq_subscribe("cmd", qos=2)
        # 定义 MQTT 消息回调
        def on_cmd_msg(client, userdata, msg):
            process_cmd(msg.payload)
        _mq_client.on_message = on_cmd_msg
        rospy.loginfo("[MQ] MQTT listening nav/+/cmd")
        while self._run and not rospy.is_shutdown():
            time.sleep(0.5)

    def _run_amqp(self):
        import pika as pk
        params = pk.ConnectionParameters(host=MQ_HOST, port=MQ_PORT, virtual_host=MQ_VHOST,
            credentials=pk.PlainCredentials(MQ_USER, MQ_PASS), heartbeat=30)
        conn = pk.BlockingConnection(params); ch = conn.channel()
        ch.exchange_declare(exchange=MQ_EXCHANGE, exchange_type="topic", durable=True)
        q = f"nav.{MQ_CLIENT_ID}.cmd.q"
        ch.queue_declare(queue=q, durable=True)
        ch.queue_bind(exchange=MQ_EXCHANGE, queue=q, routing_key=CMD_TOPIC)
        def on_msg(ch, method, props, body):
            process_cmd(body)
        ch.basic_consume(queue=q, on_message_callback=on_msg, auto_ack=True)
        rospy.loginfo(f"[MQ] AMQP listening {CMD_TOPIC}")
        while self._run and not rospy.is_shutdown() and conn.is_open:
            conn.process_data_events(time_limit=1)
        try: conn.close()
        except: pass

    def start(self):
        t = threading.Thread(target=self.run, daemon=True); t.start()
consumer = Consumer()

def _read_active_map():
    """启动时从 active/current.txt 读取当前加载的地图，并校验是否在 DB 中"""
    try:
        active_txt = os.path.join(NAV_ROOT, "lite_cog/system/map/active/current.txt")
        name = ""
        with open(active_txt) as f:
            for line in f:
                if line.startswith("NAME="):
                    name = line.strip().split("=", 1)[1]
                    break
        if not name:
            return ""
        # 校验：该地图名必须在数据库中存在，否则视为无效
        maps = get_map_list()
        valid = any(m["name"] == name for m in maps)
        if valid:
            return name
        else:
            rospy.logwarn(f"[MQ] active map '{name}' not found in DB, ignoring")
            return ""
    except: pass
    return ""

def main():
    global init_pub
    rospy.init_node("mq_adapter"); rospy.loginfo(f"[MQ] starting... (type={MQ_TYPE})")
    _load_teb_tolerances()
    mq_connect()
    init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    mb.connect()
    with state.lock: state.current_map = _read_active_map()
    rospy.loginfo(f"[MQ] current map: {state.current_map or '(none)'}")
    consumer.start(); pub.start()
    rospy.spin()

if __name__ == "__main__": main()
