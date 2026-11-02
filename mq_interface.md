# 导航系统 MQ 接口文档

> v3.0 | 2026-07-28 | MQTT 默认，兼容 AMQP

---

## 1. Topic 定义

以下 Topic 使用 MQTT 写法。AMQP 模式下将 `/` 替换为 `.`，例如
`nav/{robot_id}/status` 对应路由键 `nav.{robot_id}.status`。

| 方向 | MQTT Topic | MQTT QoS | 说明 | 状态 |
|------|-------|-----|------|------|
| 外部→机器人 | `nav/{robot_id}/cmd` | 2 | 导航 + 运控指令 | ✅ |
| 机器人→外部 | `nav/{robot_id}/cmd.ack` | 1 | 指令确认 | ✅ |
| 机器人→外部 | `nav/{robot_id}/map_list` | 1 | 地图列表 + Web 切换通知 | ✅ |
| 机器人→外部 | `nav/{robot_id}/status` | 1 | 导航状态，变化时发送，最长 2s 强制刷新 | ✅ |
| 机器人→外部 | `nav/{robot_id}/pose` | 1 | 实时位姿，变化时发送，最长 5s 强制刷新 | ✅ |
| 机器人→外部 | `nav/{robot_id}/route` | 1 | 全局路径 | ✅ |
| 机器人→外部 | `nav/{robot_id}/local_route` | 1 | TEB 局部路径 | ✅ |
| 机器人→外部 | `nav/{robot_id}/nav_points` | 1 | 当前目标点或多点任务点 | ✅ |
| 机器人→外部 | `nav/{robot_id}/heartbeat` | 1 | 心跳 (5s) | ✅ |

`{robot_id}` 对应 `MQ_CLIENT_ID`。MQTT 可使用 `nav/+/cmd` 订阅所有机器人指令；
AMQP 可使用 `nav.*.cmd`。

---

## 2. 通用消息格式

```json
{
  "header": {
    "msg_type": "nav_cmd | cmd_ack | map_list | nav_status | nav_pose | nav_route | nav_local_route | nav_points | heartbeat",
    "msg_id": "optional-request-id"
  },
  "body": { }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `msg_type` | string | 消息类型 |
| `msg_id` | string | 指令消息可选 ID；存在时由 `cmd.ack.body.ref_msg_id` 回显 |
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
  "header": {
    "msg_type": "cmd",
    "msg_id": "0fa8f5df-29c6-4ae0-a08c-54e15f68d812"
  },
  "body": {
    "cmd": "nav_single",
    "map_id": "map-001",
    "goal": { "x": 12.34, "y": -5.67, "yaw": 1.57, "frame_id": "map" }
  }
}
```

> `map_id` 可选，传入时先切换地图并发布 `initialpose` 到原点再执行导航。完整流程：切换地图 → 启动导航栈 → 等待 move_base → 通过 gateway 发 goal。

### 3.3 单点导航（轻量） ✅

> 直接将目标点发给 `move_base`，**不**切换地图、**不**通过 gateway
> 启停导航栈、**不**等待导航栈就绪。发送后仍会调用 gateway 同步 Web
> 端目标点显示。适用于导航栈已运行、只需更新目标点的场景。

```json
{
  "header": {
    "msg_type": "cmd",
    "msg_id": "42e938fa-cfbc-49ec-a2e4-23d2cf88de21"
  },
  "body": {
    "cmd": "nav_goal",
    "goal": { "x": 0.23, "y": 1.86, "yaw": 1.57, "frame_id": "map" }
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `goal.x` | number | 是 | 目标 X（米） |
| `goal.y` | number | 是 | 目标 Y（米） |
| `goal.yaw` | number | 是 | 目标朝向（弧度） |
| `goal.frame_id` | string | 否 | 坐标系，默认 `map` |

### 3.4 单点导航（仅启动） ✅

```json
{
  "header": { "msg_type": "cmd", "msg_id": "nav-only-001" },
  "body": { "cmd": "nav_only" }
}
```

使用当前地图，通过 Gateway 的 `nav-only` 命令启动单点导航模式。
不需要也不发送 `goal`，不启动多点任务，不切换地图，不额外启动雷达或运控。
导航就绪后，可通过 `nav_goal` 单独发送目标点。
Gateway 接受启动请求后返回 `cmd.ack`，其中 `cmd` 为 `nav_only`、`result`
为 `accepted`；调用失败则返回 `rejected` 和失败原因。
`accepted` 表示启动请求已接受，不代表定位及导航已就绪。

### 3.5 多点导航 ✅

```json
{
  "body": {
    "cmd": "nav_multi",
    "map_id": "map-001",
    "waypoints": [
      { "id": "wp-01", "x": 5.0, "y": 2.0, "yaw": 0.0, "stay_ms": 0 },
      { "id": "wp-02", "x": 8.5, "y": 3.2, "yaw": 1.57, "stay_ms": 5000 }
    ]
  }
}
```

> `map_id` 可选，传入时先切换地图并发布 `initialpose` 到原点再执行导航。实际流程：waypoints 写入 Task data 目录 → 启动 Task.py → 循环调用 move_base。

### 3.6 暂停 / 继续 / 取消 ✅

```json
{ "body": { "cmd": "nav_pause" } }
{ "body": { "cmd": "nav_resume" } }
{ "body": { "cmd": "nav_cancel" } }
```

### 3.7 重定位 ✅

> 发布 `initialpose` 到指定坐标，触发 AMCL/FAST-LIO 重定位。不需要切换地图。

```json
{
  "body": {
    "cmd": "relocalize",
    "pose": { "x": 2.35, "y": 3.18, "yaw": 1.57 },
    "frame_id": "map"
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `pose.x/y` | number | 是 | 当前位置（米） |
| `pose.yaw` | number | 是 | 朝向（弧度） |
| `frame_id` | string | 否 | 坐标系，默认 `map` |

### 3.8 地图切换 ✅

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

**响应**: `cmd.ack` 确认 + 推送更新后的 `map_list`

### 3.9 运控启动 / 停止 ✅

> 通过 Gateway HTTP API 间接执行。Web 端运控按钮状态实时同步。

```json
{ "body": { "cmd": "motor_start" } }
{ "body": { "cmd": "motor_stop" } }
```

### 3.10 姿态与瞬时速度控制 ✅

```json
{ "body": { "cmd": "motor_stand" } }
{ "body": { "cmd": "motor_sit" } }
{ "body": { "cmd": "motor_damp" } }
{ "body": { "cmd": "motor_move", "vx": 0.3, "vy": 0.0, "vyaw": 0.2 } }
```

| 指令/字段 | 类型 | 说明 |
|------|------|------|
| `motor_stand` | command | 通过 `/go2/sport_cmd` 发送 `stand_up` |
| `motor_sit` | command | 通过 `/go2/sport_cmd` 发送 `sit` |
| `motor_damp` | command | 通过 `/go2/sport_cmd` 发送 `damp` |
| `motor_move.vx` | number | 机器人前后速度，单位 m/s，默认 0 |
| `motor_move.vy` | number | 机器人侧向速度，单位 m/s，默认 0 |
| `motor_move.vyaw` | number | 机器人角速度，单位 rad/s，默认 0 |

`motor_move` 只发布一帧 `/cmd_vel`，属于瞬时控制指令，不会持续保活。

### 指令字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `cmd` | string | 是 | `nav_only` `nav_single` `nav_goal` `nav_multi` `switch_map` `nav_pause` `nav_resume` `nav_cancel` `map_list` `relocalize` `motor_start` `motor_stop` `motor_stand` `motor_sit` `motor_damp` `motor_move` |
| `goal.x/y/yaw` | number | — | 目标坐标(米)/朝向(弧度) |
| `goal.frame_id` | string | 否 | `nav_goal` 使用的坐标系，默认 `map`；`nav_single` 固定使用 `map` |
| `waypoints[].id` | string | — | 途经点标识 |
| `waypoints[].x/y/yaw` | number | — | 途经点坐标/朝向 |
| `waypoints[].stay_ms` | number | 否 | 保留字段；当前适配器未应用停留时间 |
| `map_id` | string | 否 | 地图 ID，`nav_single`/`nav_multi` 传入时先切换地图再导航 |
| `map_name` | string | 否 | 仅 `switch_map` 支持，与 `map_id` 二选一 |
| `pose.x/y/yaw` | number | — | 重定位目标坐标/朝向 |
---

## 4. 指令确认（cmd.ack）

`nav/{robot_id}/cmd.ack`

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

有效的 `nav_goal` / `nav_single` 指令会先发布 `accepted` ACK，再下发导航目标。
因此，同一指令触发的全局路径消息不会先于该 ACK 发布。

---

## 5. 路线数据（route / local_route）✅

### 5.1 全局路径（route）

`nav/{robot_id}/route`

```json
{
  "header": { "msg_type": "nav_route" },
  "body": {
    "route_id": "uuid",
    "frame_id": "map",
    "ref_cmd_id": "nav-command-msg-id",
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
| `frame_id` | string | 是 | 路径坐标系，来自 ROS `Path.header.frame_id`，空值时为 `map` |
| `ref_cmd_id` | string | 否 | 触发本次规划的 `nav_goal`/`nav_single` 指令 `header.msg_id` |
| `source` | object | 是 | 全局规划路径第一项的位置和朝向 |
| `target` | object | 是 | 全局规划路径最后一项的位置和朝向 |
| `path` | array[{x,y}] | 是 | 路径点序列 |
| `path_length` | number | 是 | 按完整全局路径计算的总长（米） |

全局路径最多发布 200 个均匀采样点，并始终保留第一点和最后一点。
路径完整几何发生变化时发布，因此首尾相同但中间路线变化的重规划也会发送。
`source`、`target`、`path` 和 `path_length` 均来自同一条全局规划结果，不受
雷达与 `base_link` 偏移、实时 TF 查询或历史目标状态影响。

`ref_cmd_id` 只出现在 MQ `nav_goal` 或 `nav_single` 对应的第一条全局路径中。
同一目标后续重规划发布的全局路径不包含该字段；Web 手动目标、`nav_multi`
和所有局部路径也不包含该字段。

### 5.2 局部路径（local_route）

`nav/{robot_id}/local_route`

```json
{
  "header": { "msg_type": "nav_local_route" },
  "body": {
    "route_id": "uuid",
    "source": { "x": -1.2, "y": 3.4, "yaw": 0.0 },
    "target": { "x": 0.8, "y": 3.3, "yaw": -0.05 },
    "path": [
      { "x": -1.2, "y": 3.4 },
      { "x": -0.7, "y": 3.38 },
      { "x": 0.8, "y": 3.3 }
    ],
    "path_length": 2.01
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `route_id` | string | 是 | 本次局部路径消息 ID |
| `source` | object | 是 | 发布时机器人 `base_link` 的位置和 yaw |
| `target` | object | 是 | 局部路径末点；yaw 为局部路径最后一段的方向 |
| `path` | array[{x,y}] | 是 | TEB 当前局部轨迹的全部路径点 |
| `path_length` | number | 是 | 当前局部路径总长（米） |

局部路径来自 ROS Topic `/move_base/TebLocalPlannerROS/local_plan`，仅在路径长度
或首尾点发生变化时发布。起点 `PATH_ALIGNING` 和终点 `GOAL_ALIGNING` 由 PID
接管，不运行 TEB，因此这两个阶段不会发布新的非空局部路径；进入 `ACTIVE`
后才恢复发布 TEB 局部路径。

### 5.3 目标点（nav_points）

`nav/{robot_id}/nav_points`

```json
{
  "header": { "msg_type": "nav_points" },
  "body": {
    "points": [
      { "id": "goal", "x": 6.3, "y": -0.3, "yaw": 0.0 }
    ]
  }
}
```

`nav_single` 和 `nav_goal` 发布一个 `id=goal` 的点；`nav_multi` 发布全部任务点。

---

## 6. 导航状态（status）✅

`nav/{robot_id}/status`

```json
{
  "header": { "msg_type": "nav_status" },
  "body": {
    "nav_state": "运动中",
    "current_cmd": "nav_single",
    "current_cmd_id": "",
    "current_map": "320_new"
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `nav_state` | string | 见下方状态表 |
| `current_cmd` | string | 当前指令类型 |
| `current_cmd_id` | string | 当前实现固定为空字符串 |
| `current_map` | string | 当前地图名称 |

### 6.1 状态值

| `nav_state` | 含义 |
|------|------|
| `idle` | gateway 空闲，当前没有活动导航 |
| `运动中` | 起点路径朝向 PID、TEB 路径跟踪等正常运动阶段 |
| `对齐中` | 已进入终点位置/yaw PID 对齐阶段 |
| `阻塞` | 运动中或对齐中的阻塞条件成立 |
| `到达` | `move_base` 返回 `SUCCEEDED`，或位置、yaw 和静止条件均满足 |
| `paused` | 导航暂停 |
| `stopped` | gateway 导航已停止或取消 |
| `loc_lost` | 导航中超过 3s 未收到定位相关 voxel 数据 |

状态变化时立即发布；状态不变时每 2s 强制发布一次。

### 6.2 对齐与到达判定

- TEB 状态 `PATH_ALIGNING` 和 `ACTIVE` 对外发布为 `运动中`。
- TEB 状态 `GOAL_ALIGNING` 对外发布为 `对齐中`。
- `move_base` 的 `SUCCEEDED` 是最高优先级到达信号。
- 独立判定到达时，位置误差必须不大于 `xy_goal_tolerance`，yaw 误差必须
  不大于 `yaw_goal_tolerance`，并且机器人已经静止。
- 两个容差每秒检查一次
  `lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml` 的修改时间，
  因此与 Web 参数面板持久化的到达精度保持同步。
- 到达状态会保持到收到不同的新目标，避免定位轻微漂移造成状态反复切换。

### 6.3 阻塞判定

- 只有控制器处于 `PATH_ALIGNING`、`ACTIVE` 或 `GOAL_ALIGNING`，且上一 MQ
  状态为 `运动中`、`对齐中` 或 `阻塞` 时，才允许进入 `阻塞`。
- 收到目标后 10s 仍无全局路径，报告 `阻塞`。
- 收到目标后 10s 未产生至少 0.5m 平移或 0.3rad 旋转，报告 `阻塞`。
- 已起步后使用最近至少 3s 的位姿窗口判断运动进展；普通运动主要检查线速度，
  对齐阶段同时检查线速度和角速度。
- 一般卡死条件需持续 10s 才报告，条件恢复后阻塞计时清零。

---

## 7. 实时位姿（pose）✅

`nav/{robot_id}/pose`

```json
{
  "header": { "msg_type": "nav_pose" },
  "body": {
    "frame_id": "map",
    "position": { "x": 2.35, "y": 3.18, "z": 0.0 },
    "orientation": { "roll": 0.0, "pitch": 0.0, "yaw": 1.57 },
    "localization_quality": "good",
    "location_status": "就绪"
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `frame_id` | string | 坐标系，默认 `map`（TF_MAP_FRAME 环境变量可配） |
| `position.x/y/z` | number | 坐标（米） |
| `orientation.roll/pitch/yaw` | number | 欧拉角（弧度） |
| `localization_quality` | string | 当前实现固定为 `good`，兼容字段 |
| `location_status` | string | `就绪`：3s 内有 voxel 数据；`仅TF`：TF 有效但 voxel 超时 |

发布循环为 1Hz；位置变化达到 0.05m、yaw 变化达到 0.05rad 时发布，否则每
5s 强制发布一次。TF 查询失败时本轮不发布 pose；导航期间 voxel 超过 3s
未更新会通过 status 发布 `loc_lost`。

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
| 单点导航（仅启动） | ✅ | `nav_only`：使用当前地图启动单点导航模式，不发目标点 |
| 多点导航 | ✅ | 写 waypoint JSON → 启动 `Task.py` |
| 暂停 | ✅ | cancel move_base goal |
| 继续 | ✅ | 重发上次 goal |
| 取消 | ✅ | cancel goal + kill Task.py |
| 重定位 | ✅ | 发布 `initialpose` 到指定坐标 |
| 运控启动/停止 | ✅ | MQ → Gateway HTTP API，Web 端同步按钮状态 |
| 姿态/速度控制 | ✅ | stand/sit/damp + 单帧全向速度指令 |
| 导航状态推送 | ✅ | 状态变化立即发布，最长 2s 强制刷新 |
| 位姿推送 | ✅ | 变化触发，最长 5s 强制刷新 |
| 路线推送 | ✅ | 订阅 global plan → `nav/{id}/route` |
| 局部路线推送 | ✅ | 订阅 TEB local plan → `nav/{id}/local_route` |
| 目标点推送 | ✅ | 单点/多点指令 → `nav/{id}/nav_points` |
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

---

## 11. 环境变量

```bash
# ---- MQ ----
MQ_TYPE=mqtt            # mqtt（默认）| rabbitmq
MQ_HOST=<broker-host>
MQ_PORT=1883            # rabbitmq 通常为 5672
MQ_USER=<username>
MQ_PASS=<password>
MQ_VHOST=/
MQ_EXCHANGE=nav.exchange
MQ_CLIENT_ID=<robot-id>

# ---- TF 帧名 ----
TF_MAP_FRAME=map
TF_BODY_FRAME=base_link
TEB_CONFIG=/home/unitree/go2_nav/lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml

# ---- Gateway (MQ → HTTP) ----
GATEWAY_URL=http://127.0.0.1:8080
GATEWAY_USER=admin
GATEWAY_PASS=admin123
```

**说明**:
- `mq_adapter.py` 默认使用 MQTT，也可通过 `MQ_TYPE=rabbitmq` 切换为 AMQP。
- Gateway 当前通过 AMQP 订阅 `status`、`pose`、`route` 和 `nav_points` 并推送至 Web。
