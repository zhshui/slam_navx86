#!/usr/bin/env python3
"""发送 MQ 消息 — 支持 MQTT 和 RabbitMQ

用法：直接修改下面 MSG 的内容，然后 python3 mq_send.py 即可。
"""

import sys
import os
import json

# ═══════════════════════════════════════════════════════════════
#  改这里 ↓ 
# ═══════════════════════════════════════════════════════════════
MSG = json.dumps({
  "header": { "msg_type": "cmd" },
  "body": {
    "cmd": "switch_map",
    "map_id": "00"
  }
  })
# ═══════════════════════════════════════════════════════════════

# ── MQ 配置 ──────────────────────────────────────────────────
MQ_TYPE = os.environ.get("MQ_TYPE", "mqtt")
MQ_HOST = os.environ.get("MQ_HOST", "49.233.183.203")
MQ_PORT = int(os.environ.get("MQ_PORT", "1883"))
MQ_USER = os.environ.get("MQ_USER", "server")
MQ_PASS = os.environ.get("MQ_PASS", "5bP!8aS3$kD7vF2&")
MQ_CLIENT_ID = os.environ.get("MQ_CLIENT_ID", "B42D1000P3499GGW")
EXCHANGE = os.environ.get("MQ_EXCHANGE", "nav.exchange")
VHOST = os.environ.get("MQ_VHOST", "/")

if MQ_TYPE == "mqtt":
    import uuid
    import paho.mqtt.client as mqtt_lib

    SENDER_ID = f"{MQ_CLIENT_ID}-send-{uuid.uuid4().hex[:8]}"
    client = mqtt_lib.Client(client_id=SENDER_ID, protocol=mqtt_lib.MQTTv311)
    client.username_pw_set(MQ_USER, MQ_PASS)
    client.connect(MQ_HOST, MQ_PORT, keepalive=30)
    client.loop_start()
    topic = f"nav/{MQ_CLIENT_ID}/cmd"
    info = client.publish(topic, MSG, qos=1)
    info.wait_for_publish(timeout=3)
    print(f"📤 MQTT {topic} ← {MSG}")
    client.loop_stop()
    client.disconnect()
else:
    import pika

    params = pika.ConnectionParameters(
        host=MQ_HOST, port=MQ_PORT, virtual_host=VHOST,
        credentials=pika.PlainCredentials(MQ_USER, MQ_PASS), heartbeat=30,
    )
    conn = pika.BlockingConnection(params)
    ch = conn.channel()
    ch.exchange_declare(exchange=EXCHANGE, exchange_type="topic", durable=True)
    ch.basic_publish(
        exchange=EXCHANGE,
        routing_key=f"nav.{MQ_CLIENT_ID}.cmd",
        body=MSG,
    )
    print(f"📤 AMQP nav.{MQ_CLIENT_ID}.cmd ← {MSG}")
    conn.close()
