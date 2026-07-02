# 导航系统 MQ 接口文档

> v2.4 | 2026-06-30 | AMQP 默认；

---

## 1. Topic 定义

| 方向 | Topic | QoS | 说明 | 状态 |
|------|-------|-----|------|------|
| 外部→机器人 | `nav/{robot_id}/cmd` | 2 | 导航 + 运控指令 | ✅ |
| 机器人→外部 | `nav/{robot_id}/cmd/ack` | 2 | 指令确认 | ✅ |
| 机器人→外部 | `nav/{robot_id}/map_list` | 2 | 地图列表 + Web 切换通知 | ✅ |
| 机器人→外部 | `nav/{robot_id}/status` | 1 | 导航状态 (1Hz) | ✅ |
| 机器人→外部 | `nav/{robot_id}/pose` | 1 | 实时位姿 (1Hz) | ✅ |
| 机器人→外部 | `nav/{robot_id}/route` | 2 | 路线数据 | ✅ |
| 机器人→外部 | `nav/{robot_id}/heartbeat` | 1 | 心跳 (5s) | ✅ |

`{robot_id}` 对应 `MQ_CLIENT_ID`，支持通配符 `nav/+/cmd`。

---

## 2. 通用消息格式

```json
{
  "header": {
    "msg_type": "nav_cmd | cmd_ack | map_list | nav_status | nav_pose | nav_route | heartbeat"
  },
  "body": { }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `msg_type` | string | 消息类型 |
| `body` | object | 消息体 |

---

## 3. 指令（cmd）

`nav/{robot_id}/cmd`

### 3.1 地图列表查询 ✅

```json
{ "body": { "cmd": "map_list" } }
```

**响应** (发布到 `nav/{robot_id}/map_list`):

```json
{
  "header": { "msg_type": "map_list" },
  "body": {
    "maps": [
      {
        "id": "uuid",
        "name": "scans_001",
        "yaml_path": "data/maps/scans_001.yaml",
        "pcd_path": "data/maps/scans_001.pcd",
        "created_at": "2026-06-05T10:00:00",
        "active": true
      }
    ],
    "total": 1
  }
}
```

### 3.2 单点导航（完整） ✅

```json
{
  "body": {
    "cmd": "nav_single",
    "map_id": "map-001",
    "goal": { "x": 12.34, "y": -5.67, "yaw": 1.57, "frame_id": "camera_init" },
    "options": { "timeout_ms": 60000, "retry_count": 0 }
  }
}
```

> `map_id` 可选，传入时先切换地图并发布 `initialpose` 到原点再执行导航。完整流程：切换地图 → 启动导航栈 → 等待 move_base → 通过 gateway 发 goal。

### 3.3 单点导航（轻量） ✅

> 直接将目标点发给 `move_base`，**不**切换地图、**不**通过 gateway 编排、**不**等待导航栈就绪。适用于导航栈已运行、只需更新目标点的场景。

```json
{
  "body": {
    "cmd": "nav_goal",
    "goal": { "x": 12.34, "y": -5.67, "yaw": 1.57, "frame_id": "map" }
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `goal.x` | number | 是 | 目标 X（米） |
| `goal.y` | number | 是 | 目标 Y（米） |
| `goal.yaw` | number | 是 | 目标朝向（弧度） |
| `goal.frame_id` | string | 否 | 坐标系，默认 `map` |

### 3.4 多点导航 ✅

```json
{
  "body": {
    "cmd": "nav_multi",
    "map_id": "map-001",
    "waypoints": [
      { "id": "wp-01", "x": 5.0, "y": 2.0, "yaw": 0.0, "stay_ms": 0 },
      { "id": "wp-02", "x": 8.5, "y": 3.2, "yaw": 1.57, "stay_ms": 5000 }
    ],
    "options": { "loop": False }
  }
}
```

> `map_id` 可选，传入时先切换地图并发布 `initialpose` 到原点再执行导航。实际流程：waypoints 写入 Task data 目录 → 启动 Task.py → 循环调用 move_base。

### 3.5 暂停 / 继续 / 取消 ✅

```json
{ "body": { "cmd": "nav_pause" } }
{ "body": { "cmd": "nav_resume" } }
{ "body": { "cmd": "nav_cancel" } }
```

### 3.6 重定位 ✅

> 发布 `initialpose` 到指定坐标，触发 AMCL/FAST-LIO 重定位。不需要切换地图。

```json
{
  "body": {
    "cmd": "relocalize",
    "pose": { "x": 2.35, "y": 3.18, "yaw": 1.57 },
    "frame_id": "camera_init"
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `pose.x/y` | number | 是 | 当前位置（米） |
| `pose.yaw` | number | 是 | 朝向（弧度） |
| `frame_id` | string | 否 | 坐标系，默认 `camera_init` |

### 3.7 地图切换 ✅

> 独立的地图切换指令。切换后发布 `initialpose` 到原点，并推送更新后的 `map_list`。不需要导航。

```json
{
  "body": {
    "cmd": "switch_map",
    "map_id": "map-uuid"
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `map_id` | string | 二选一 | 地图 ID（与 `map_name` 二选一） |
| `map_name` | string | 二选一 | 地图名称（与 `map_id` 二选一） |

**响应**: `cmd/ack` 确认 + 推送更新后的 `map_list`

### 3.8 运控启动 / 停止 ✅

> 通过 Gateway HTTP API 间接执行。Web 端运控按钮状态实时同步。

```json
{ "body": { "cmd": "motor_start" } }
{ "body": { "cmd": "motor_stop" } }
```

### 指令字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `cmd` | string | 是 | `nav_single` `nav_goal` `nav_multi` `switch_map` `nav_pause` `nav_resume` `nav_cancel` `map_list` `relocalize` `motor_start` `motor_stop` |
| `goal.x/y/yaw` | number | — | 目标坐标(米)/朝向(弧度) |
| `goal.frame_id` | string | 否 | 坐标系，默认 `camera_init` |
| `goal.yaw_tolerance` | number | 否 | 朝向容差，默认 0.087 |
| `waypoints[].id` | string | — | 途经点标识 |
| `waypoints[].x/y/yaw` | number | — | 途经点坐标/朝向 |
| `waypoints[].stay_ms` | number | 否 | 停留时间(毫秒) |
| `options.timeout_ms` | number | 否 | 超时(毫秒) |
| `options.retry_count` | number | 否 | 重试次数 |
| `options.loop` | boolean | 否 | 多点是否循环 |
| `map_id` | string | 否 | 地图 ID，`nav_single`/`nav_multi` 传入时先切换地图再导航 |
| `map_name` | string | 否 | 地图名称（与 `map_id` 二选一，`nav_single`/`nav_multi` 传入时先切换地图再导航） |
| `pose.x/y/yaw` | number | — | 重定位目标坐标/朝向 |
---

## 4. 指令确认（cmd/ack）

`nav/{robot_id}/cmd/ack`

```json
{
  "body": {
    "ref_msg_id": "a1b2c3d4-...",
    "cmd": "nav_single",
    "result": "accepted",
    "reason": ""
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `ref_msg_id` | string | 关联指令 msg_id |
| `cmd` | string | 回显指令类型 |
| `result` | string | `accepted` / `rejected` |
| `reason` | string | 拒绝原因（rejected 时） |

---

## 5. 路线数据（route）✅

`nav/{robot_id}/route`

```json
{
  "body": {
    "route_id": "uuid",
    "source": { "x": -1.2, "y": 3.4, "yaw": 0.0 },
    "target": { "x": 12.34, "y": -5.67, "yaw": 1.57 },
    "path": [
      { "x": -1.2, "y": 3.4 },
      { "x": -0.8, "y": 3.38 }
    ],
    "path_length": 18.72
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `route_id` | string | 是 | 路线 ID |
| `source` | object | 是 | 起点（当前位置+朝向） |
| `target` | object | 是 | 终点（目标坐标+朝向） |
| `path` | array[{x,y}] | 是 | 路径点序列 |
| `path_length` | number | 是 | 路径总长（米） |

---

## 6. 导航状态（status）✅

`nav/{robot_id}/status` (1Hz)

```json
{
  "header": { "msg_type": "nav_status" },
  "body": {
    "nav_state": "running",
    "current_cmd": "nav_single",
    "current_cmd_id": "uuid"
    "current_map": "320_new"
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `nav_state` | string | `idle` `running` `paused` `completed` `failed` `cancelled` |
| `current_cmd` | string | 当前指令类型 |
| `current_cmd_id` | string | 当前指令 msg_id |

---

## 7. 实时位姿（pose）✅

`nav/{robot_id}/pose` (1Hz)

```json
{
  "header": { "msg_type": "nav_pose" },
  "body": {
    "frame_id": "map",
    "position": { "x": 2.35, "y": 3.18, "z": 0.0 },
    "orientation": { "roll": 0.0, "pitch": 0.0, "yaw": 1.57 },
    "localization_quality": "good"
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `frame_id` | string | 坐标系，默认 `map`（TF_MAP_FRAME 环境变量可配） |
| `position.x/y/z` | number | 坐标（米） |
| `orientation.roll/pitch/yaw` | number | 欧拉角（弧度） |
| `localization_quality` | string | `good` `degraded` `lost` |

---

## 8. 心跳（heartbeat）✅

`nav/{robot_id}/heartbeat`，间隔 5s，超时 15s 离线。

```json
{
  "header": { "msg_type": "heartbeat" },
  "body": { "online": true, "nav_state": "idle" }
}
```

---

## 9. 异常处理

- 指令 500ms 无 ack 视为失败，重试 3 次，间隔 200ms

---

## 10. 实现现状

| 功能 | MQ | 说明 |
|------|:--:|------|
| 地图列表查询 | ✅ | 读取 gateway SQLite 数据库 |
| 地图切换（附带导航） | ✅ | `nav_single`/`nav_multi` 传 `map_id` 时自动切换 |
| 地图切换（独立） | ✅ | `switch_map` 单独切图，推送更新后的 `map_list` |
| 单点导航 | ✅ | 完整流程：切图 → 启栈 → gateway 发 goal |
| 单点导航（轻量） | ✅ | 直接发 goal 给 `move_base`，不切图不走 gateway |
| 多点导航 | ✅ | 写 waypoint JSON → 启动 `Task.py` |
| 暂停 | ✅ | cancel move_base goal |
| 继续 | ✅ | 重发上次 goal |
| 取消 | ✅ | cancel goal + kill Task.py |
| 重定位 | ✅ | 发布 `initialpose` 到指定坐标 |
| 运控启动/停止 | ✅ | MQ → Gateway HTTP API，Web 端同步按钮状态 |
| 导航状态推送 | ✅ | 1Hz 定时发布 `nav/{id}/status` |
| 位姿推送 | ✅ | 1Hz TF→MQ 发布 `nav/{id}/pose` |
| 路线推送 | ✅ | 订阅 global plan → `nav/{id}/route` |
| 心跳 | ✅ | 5s 间隔发布 `nav/{id}/heartbeat` |

**节点**: `lite_cog/system/scripts/mq/mq_adapter.py`  
**Gateway**: `ros_web_gui_app/gateway/src/index.ts`

**启动**:
```bash
# 方式1: 直接运行
python3 lite_cog/system/scripts/mq/mq_adapter.py

# 方式2: roslaunch
roslaunch lite_cog/system/scripts/mq/mq_adapter.launch
```

**测试**:
```bash
# 单元测试 (不依赖 MQ/ROS) — 导航 + 运控全覆盖
python3 lite_cog/system/scripts/mq/test_mq.py   # 20/20 passed

# 交互式测试工具 (需要 RabbitMQ + gateway + mq_adapter)
python3 lite_cog/system/scripts/mq/mq_tester.py

# 集成测试 (需要 RabbitMQ + ROS)
# 已验证: map_list, nav_single, nav_goal, nav_multi, switch_map, nav_pause, nav_cancel,
#         relocalize, motor_start, motor_stop, status/pose/route 推送, 未知指令拒绝
```

---

## 11. 环境变量

```bash
# ---- RabbitMQ ----
MQ_TYPE=rabbitmq        # rabbitmq | mqtt
MQ_HOST=127.0.0.1
MQ_PORT=5672            # mqtt 默认 1883
MQ_USER=nav
MQ_PASS=nav123
MQ_VHOST=/
MQ_EXCHANGE=nav.exchange
MQ_CLIENT_ID=robot-001

# ---- TF 帧名 ----
TF_MAP_FRAME=map
TF_BODY_FRAME=base_link

# ---- Gateway (MQ → HTTP) ----
GATEWAY_URL=http://127.0.0.1:8080
GATEWAY_USER=admin
GATEWAY_PASS=admin123
```

**说明**:
- Gateway 启动时自动连接 RabbitMQ，订阅 status/pose/route 主题推送至 Web 前端
