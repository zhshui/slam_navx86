#!/usr/bin/env python3
"""监听所有导航 MQ 消息，支持 AMQP/MQTT 自动切换
用法:
  python3 mq_listen.py                 # 监听所有
  python3 mq_listen.py map_list        # 只看 map_list
  python3 mq_listen.py status          # 只看 status
"""
import json, time, os, sys
MQ_TYPE = os.environ.get("MQ_TYPE", "mqtt")
MQ_HOST = os.environ.get("MQ_HOST", "49.233.183.203")
_DEFAULT_PORTS = {"rabbitmq": 5672, "mqtt": 1883}
MQ_PORT = int(os.environ.get("MQ_PORT", _DEFAULT_PORTS.get(MQ_TYPE, 1883)))
MQ_USER = os.environ.get("MQ_USER", "server")
MQ_PASS = os.environ.get("MQ_PASS", "5bP!8aS3$kD7vF2&")
MQ_VHOST = os.environ.get("MQ_VHOST", "/")
MQ_EXCHANGE = os.environ.get("MQ_EXCHANGE", "nav.exchange")
MQ_CLIENT_ID = os.environ.get("MQ_CLIENT_ID", "B42D1000P3499GGW")

filter_type = sys.argv[1] if len(sys.argv) > 1 else None

ICONS = {
    "cmd": "📤", "cmd.ack": "📨", "cmd_ack": "📨",
    "status": "📊", "pose": "📍", "map_list": "📂",
    "heartbeat": "💓", "route": "🗺",
}

def show(topic, data):
    sep = "/" if MQ_TYPE == "mqtt" else "."
    topic_suffix = topic.split(sep)[-1]
    if filter_type and topic_suffix != filter_type:
        return
    icon = ICONS.get(topic_suffix, "📡")
    bar = "─" * 60
    print(f"\n{bar}")
    print(f"{icon} {topic}")
    print(json.dumps(data, ensure_ascii=False, indent=2))
    print(bar)

if MQ_TYPE == "mqtt":
    import paho.mqtt.client as mqtt
    _connected = False
    _connect_error = None
    def on_msg(client, userdata, msg):
        try: data = json.loads(msg.payload.decode("utf-8", errors="replace"))
        except: data = msg.payload.decode("utf-8", errors="replace")
        show(msg.topic, data)
    def on_connect(client, userdata, flags, rc):
        global _connected, _connect_error
        if rc == 0:
            _connected = True
            client.subscribe(f"nav/{MQ_CLIENT_ID}/#")
            print(f"✅ MQTT 已连接 {MQ_HOST}:{MQ_PORT}")
        else:
            _connect_error = rc
            err_msg = {1:"协议版本错误",2:"ClientID 被拒",3:"服务器不可用",4:"用户名/密码错误",5:"未授权"}.get(rc, f"未知错误({rc})")
            print(f"❌ MQTT 连接失败: {err_msg}")
    client = mqtt.Client(client_id=f"{MQ_CLIENT_ID}-listen", protocol=mqtt.MQTTv311)
    client.username_pw_set(MQ_USER, MQ_PASS)
    client.on_message = on_msg
    client.on_connect = on_connect
    result = client.connect(MQ_HOST, MQ_PORT, keepalive=60)
    # paho-mqtt v1.x connect() 返回 int (0=成功), v2.x 返回 MutableConnectResult
    rc = result.rc if hasattr(result, 'rc') else result
    if rc != 0:
        print(f"❌ MQTT connect() 失败: rc={rc}")
        sys.exit(1)
    client.loop_start()
    topic_hint = f"nav/{MQ_CLIENT_ID}/{filter_type or '#'}"
    print(f"👂 MQTT 监听 {topic_hint} (Ctrl+C 退出)")
    # 等待连接建立（最多 5 秒）
    waited = 0
    while not _connected and waited < 50:
        if _connect_error is not None:
            print(f"❌ 无法连接 MQTT broker")
            sys.exit(1)
        time.sleep(0.1); waited += 0.1
    if not _connected:
        print(f"❌ 连接 MQTT broker 超时 (5s)")
        sys.exit(1)
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")
        client.loop_stop()
        client.disconnect()
else:
    import pika
    params = pika.ConnectionParameters(host=MQ_HOST, port=MQ_PORT, virtual_host=MQ_VHOST,
        credentials=pika.PlainCredentials(MQ_USER, MQ_PASS), heartbeat=30)
    conn = pika.BlockingConnection(params)
    ch = conn.channel()
    q = ch.queue_declare('', exclusive=True, auto_delete=True).method.queue
    pattern = f"nav.{MQ_CLIENT_ID}.{filter_type}" if filter_type else f"nav.{MQ_CLIENT_ID}.#"
    ch.queue_bind(exchange=MQ_EXCHANGE, queue=q, routing_key=pattern)
    print(f"👂 AMQP 监听 {MQ_EXCHANGE} / {pattern} (Ctrl+C 退出)")
    try:
        for method, props, body in ch.consume(queue=q, auto_ack=True, inactivity_timeout=1):
            if body is None: continue
            try: data = json.loads(body)
            except: data = body.decode("utf-8", errors="replace")
            show(method.routing_key, data)
    except KeyboardInterrupt:
        print("\n退出")
        conn.close()
