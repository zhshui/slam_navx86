import { execFile } from 'node:child_process'
import { promisify } from 'node:util'
import { writeFileSync, mkdirSync } from 'node:fs'
import path from 'node:path'
import { deflateSync } from 'node:zlib'
import { WebSocket } from 'ws'
import { runtimeState, LARGE_MAP_CELL_THRESHOLD } from './state.js'

const execFileAsync = promisify(execFile)

export interface RosTopics {
  map: string
  tf: string
  lidar: string
  cmdVel: string
  virtualObstacles: string
  moveBaseGoal: string
  moveBaseSimpleGoal: string
  globalPlan: string
  localPlan: string
  voxelGrid: string
  mapCloud?: string
  slamPath?: string
  cmdVelOut?: string
  jointStates?: string
  sportStatus?: string
}

export interface RosAdapterStatus {
  mode: 'mock' | 'rosbridge'
  rosbridgeUrl: string | null
  connected: boolean
  topics: RosTopics
  lastMapAt: string | null
  lastTfAt: string | null
  lastLidarAt: string | null
}

export interface RosAdapter {
  start(): Promise<void>
  saveMap(name: string): Promise<{ yamlPath: string }>
  switchMap(yamlPath: string): Promise<void>
  publishVirtualObstacles(payload: unknown): Promise<void>
  publishCmdVel(cmd: { linear: number; angular: number }): Promise<void>
  publishNavGoal(goal: { x: number; y: number; yaw: number; frameId: string }): Promise<void>
  publishOccupancyGrid(grid: { width: number; height: number; resolution: number; origin: { x: number; y: number }; data: number[] }): Promise<void>
  publishInitialPose(pose: { x: number; y: number; yaw: number; frameId: string }): Promise<void>
  publishSimpleGoal(goal: { x: number; y: number; yaw: number; frameId: string }): Promise<void>
  getStatus(): RosAdapterStatus
}

interface RosConfig {
  mode: 'mock' | 'rosbridge'
  rosbridgeUrl: string
  topics: RosTopics
  virtualObstaclesType: string
  publishSimpleGoal: boolean
}

type WsFactory = (url: string) => WebSocket

const FLOAT32 = 7
const DEFAULT_TOPICS: RosTopics = {
  map: '/map',
  tf: '/tf',
  lidar: '/livox/lidar',
  mapCloud: '/cloud_registered',
  slamPath: '/path',
  cmdVel: '/cmd_vel',
  virtualObstacles: '/virtual_obstacles',
  moveBaseGoal: '/move_base/goal',
  moveBaseSimpleGoal: '/move_base_simple/goal',
  globalPlan: '/move_base/GlobalPlanner/plan',
  localPlan: '/move_base/TebLocalPlannerROS/local_plan',
  voxelGrid: '/move_base/local_costmap/stvl_obstacle_layer/voxel_grid',
}

function readRosConfig(): RosConfig {
  const mode = process.env.ROS_MODE === 'rosbridge' ? 'rosbridge' : 'mock'
  return {
    mode,
    rosbridgeUrl: process.env.ROSBRIDGE_URL ?? 'ws://127.0.0.1:9090',
    topics: {
      map: process.env.ROS_TOPIC_MAP ?? DEFAULT_TOPICS.map,
      tf: process.env.ROS_TOPIC_TF ?? DEFAULT_TOPICS.tf,
      lidar: process.env.ROS_TOPIC_LIDAR ?? DEFAULT_TOPICS.lidar,
      cmdVel: process.env.ROS_TOPIC_CMD_VEL ?? DEFAULT_TOPICS.cmdVel,
      virtualObstacles: process.env.ROS_TOPIC_VIRTUAL_OBSTACLES ?? DEFAULT_TOPICS.virtualObstacles,
      moveBaseGoal: process.env.ROS_TOPIC_MOVE_BASE_GOAL ?? DEFAULT_TOPICS.moveBaseGoal,
      moveBaseSimpleGoal: process.env.ROS_TOPIC_MOVE_BASE_SIMPLE_GOAL ?? DEFAULT_TOPICS.moveBaseSimpleGoal,
      globalPlan: process.env.ROS_TOPIC_GLOBAL_PLAN ?? DEFAULT_TOPICS.globalPlan,
      localPlan: process.env.ROS_TOPIC_LOCAL_PLAN ?? DEFAULT_TOPICS.localPlan,
      voxelGrid: process.env.ROS_TOPIC_VOXEL_GRID ?? DEFAULT_TOPICS.voxelGrid,
    },
    virtualObstaclesType: process.env.ROS_TOPIC_VIRTUAL_OBSTACLES_TYPE ?? 'std_msgs/String',
    publishSimpleGoal: process.env.ROS_PUBLISH_MOVE_BASE_SIMPLE_GOAL === '1',
  }
}

function nowIso() {
  return new Date().toISOString()
}

function rosTime() {
  const now = Date.now()
  return {
    secs: Math.floor(now / 1000),
    nsecs: (now % 1000) * 1_000_000,
  }
}

/** CRC-32 lookup table (PNG uses CRC-32/IEEE) */
function crc32Table(): Uint32Array {
  const table = new Uint32Array(256)
  for (let n = 0; n < 256; n++) {
    let c = n
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1
    table[n] = c
  }
  return table
}
const CRC_TABLE = crc32Table()

function crc32(buf: Buffer): number {
  let c = 0xffffffff
  for (let i = 0; i < buf.length; i++) c = CRC_TABLE[(c ^ buf[i]!) & 0xff]! ^ (c >>> 8)
  return (c ^ 0xffffffff) >>> 0
}

function pngChunk(type: string, data: Buffer): Buffer {
  const typeData = Buffer.concat([Buffer.from(type, 'ascii'), data])
  const len = Buffer.alloc(4)
  len.writeUInt32BE(data.length, 0)
  const crc = Buffer.alloc(4)
  crc.writeUInt32BE(crc32(typeData), 0)
  return Buffer.concat([len, typeData, crc])
}

/** Convert OccupancyGrid data to grayscale PNG + also write raw PGM for editor fetch.
 *  PNG for display (compressed ~100x), PGM for MapEditor raw data access. */
function writeMapFiles(
  width: number, height: number, data: number[], mapDir: string
): string | null {
  try {
    mkdirSync(mapDir, { recursive: true })

    // Build raw image rows (filter byte 0 + width grayscale bytes per row)
    const rawRows = Buffer.alloc((width + 1) * height)
    for (let y = 0; y < height; y++) {
      rawRows[y * (width + 1)] = 0 // filter: None
      for (let x = 0; x < width; x++) {
        const srcIdx = (height - 1 - y) * width + x
        const v = data[srcIdx] ?? -1
        rawRows[y * (width + 1) + 1 + x] = v === 100 ? 0 : v === 0 ? 254 : 205
      }
    }

    // Write PGM (raw binary, ~40MB) — for MapEditor editing
    const pgmPixels = Buffer.alloc(width * height)
    for (let y = 0; y < height; y++) {
      rawRows.copy(pgmPixels, y * width, y * (width + 1) + 1, (y + 1) * (width + 1))
    }
    const pgmHeader = Buffer.from(`P5\n${width} ${height}\n255\n`)
    writeFileSync(path.join(mapDir, 'live_map.pgm'), Buffer.concat([pgmHeader, pgmPixels]))

    // Write PNG (compressed, ~300KB) — for display
    const sig = Buffer.from([137, 80, 78, 71, 13, 10, 26, 10])
    const ihdrData = Buffer.alloc(13)
    ihdrData.writeUInt32BE(width, 0); ihdrData.writeUInt32BE(height, 4)
    ihdrData[8] = 8; ihdrData[9] = 0; ihdrData[10] = 0; ihdrData[11] = 0; ihdrData[12] = 0
    const png = Buffer.concat([
      sig, pngChunk('IHDR', ihdrData),
      pngChunk('IDAT', deflateSync(rawRows)),
      pngChunk('IEND', Buffer.alloc(0)),
    ])
    writeFileSync(path.join(mapDir, 'live_map.png'), png)

    return `/static/maps/live_map.png?t=${Date.now()}`
  } catch (e) {
    console.error('[rosAdapter] Failed to write map files:', e)
    return null
  }
}

function yawToQuaternion(yaw: number) {
  return {
    x: 0,
    y: 0,
    z: Math.sin(yaw / 2),
    w: Math.cos(yaw / 2),
  }
}

function quaternionToYaw(q: { x?: number; y?: number; z?: number; w?: number }) {
  const x = Number(q.x ?? 0)
  const y = Number(q.y ?? 0)
  const z = Number(q.z ?? 0)
  const w = Number(q.w ?? 1)
  const siny = 2 * (w * z + x * y)
  const cosy = 1 - 2 * (y * y + z * z)
  return Math.atan2(siny, cosy)
}

function parsePointCloud2(pointsMsg: unknown): Array<{ x: number; y: number }> {
  const msg = pointsMsg as {
    fields?: Array<{ name?: string; offset?: number; datatype?: number }>
    point_step?: number
    is_bigendian?: boolean
    data?: number[] | string
    width?: number
    height?: number
  }

  const fields = Array.isArray(msg.fields) ? msg.fields : []
  const xField = fields.find((f) => f.name === 'x')
  const yField = fields.find((f) => f.name === 'y')
  const pointStep = Number(msg.point_step ?? 0)
  if (!xField || !yField || pointStep <= 0 || xField.datatype !== FLOAT32 || yField.datatype !== FLOAT32) {
    return []
  }

  let bytes: Uint8Array
  if (Array.isArray(msg.data)) {
    bytes = Uint8Array.from(msg.data)
  } else if (typeof msg.data === 'string') {
    bytes = Uint8Array.from(Buffer.from(msg.data, 'base64'))
  } else {
    return []
  }

  const xOffset = Number(xField.offset ?? 0)
  const yOffset = Number(yField.offset ?? 0)
  const littleEndian = !msg.is_bigendian
  const totalPoints = Number(msg.width ?? 0) * Number(msg.height ?? 1)
  const maxPoints = Math.min(totalPoints || Math.floor(bytes.length / pointStep), 256)

  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
  const output: Array<{ x: number; y: number }> = []
  for (let i = 0; i < maxPoints; i++) {
    const base = i * pointStep
    if (base + Math.max(xOffset, yOffset) + 4 > view.byteLength) {
      break
    }
    const x = view.getFloat32(base + xOffset, littleEndian)
    const y = view.getFloat32(base + yOffset, littleEndian)
    if (Number.isFinite(x) && Number.isFinite(y)) {
      output.push({ x, y })
    }
  }

  return output
}

function parsePath(pathMsg: unknown): Array<{ x: number; y: number }> {
  const msg = pathMsg as { poses?: Array<{ pose?: { position?: { x?: number; y?: number } } }> }
  const poses = Array.isArray(msg.poses) ? msg.poses : []
  return poses
    .map((p) => ({ x: Number(p.pose?.position?.x ?? 0), y: Number(p.pose?.position?.y ?? 0) }))
    .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y))
}

class MockRosAdapter implements RosAdapter {
  private readonly config: RosConfig

  constructor(config: RosConfig) {
    this.config = config
  }

  async start() {
    runtimeState.rosConnected = true
  }

  async saveMap(name: string) {
    if (process.env.MAP_SAVE_HOOK) {
      await execFileAsync(process.env.MAP_SAVE_HOOK, [name])
    }
    const mapDir = process.env.MAP_DIR || '/home/shui/LY/lite_cog/system/map'
    return { yamlPath: `${mapDir}/${name}.yaml` }
  }

  async switchMap(yamlPath: string): Promise<void> {
    if (process.env.MAP_SWITCH_HOOK) {
      await execFileAsync(process.env.MAP_SWITCH_HOOK, [yamlPath])
    }
    return
  }

  async publishVirtualObstacles(): Promise<void> {
    return
  }

  async publishCmdVel(): Promise<void> {
    return
  }

  async publishNavGoal(): Promise<void> {
    return
  }

  async publishOccupancyGrid(): Promise<void> {
    return
  }

  async publishInitialPose(): Promise<void> {
    return
  }

  async publishSimpleGoal(): Promise<void> {
    return
  }

  getStatus(): RosAdapterStatus {
    return {
      mode: 'mock',
      rosbridgeUrl: null,
      connected: runtimeState.rosConnected,
      topics: this.config.topics,
      lastMapAt: runtimeState.lastMapAt,
      lastTfAt: runtimeState.lastTfAt,
      lastLidarAt: runtimeState.lastLidarAt,
    }
  }
}

export class RosbridgeAdapter implements RosAdapter {
  private socket: WebSocket | null = null
  private reconnectTimer: NodeJS.Timeout | null = null
  private shouldReconnect = true
  private cmdVelAdvertised = false
  private virtualObstaclesAdvertised = false
  private moveBaseGoalAdvertised = false
  private moveBaseSimpleGoalAdvertised = false

  constructor(
    private readonly config: RosConfig,
    private readonly wsFactory: WsFactory = (url) => new WebSocket(url),
  ) {}

  async start() {
    this.shouldReconnect = true
    this.connect()
  }

  stop() {
    this.shouldReconnect = false
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    if (this.socket) {
      this.socket.close()
      this.socket = null
    }
    runtimeState.rosConnected = false
  }

  async saveMap(name: string) {
    if (process.env.MAP_SAVE_HOOK) {
      await execFileAsync(process.env.MAP_SAVE_HOOK, [name])
    }
    const mapDir = process.env.MAP_DIR || '/home/shui/LY/lite_cog/system/map'
    return { yamlPath: `${mapDir}/${name}.yaml` }
  }

  async switchMap(yamlPath: string): Promise<void> {
    if (process.env.MAP_SWITCH_HOOK) {
      await execFileAsync(process.env.MAP_SWITCH_HOOK, [yamlPath])
    }
  }

  async publishVirtualObstacles(payload: unknown): Promise<void> {
    this.ensureVirtualObstaclesAdvertised()
    const message =
      this.config.virtualObstaclesType === 'std_msgs/String' ? { data: JSON.stringify(payload) } : payload
    this.send({
      op: 'publish',
      topic: this.config.topics.virtualObstacles,
      msg: message,
    })
  }

  async publishCmdVel(cmd: { linear: number; angular: number }): Promise<void> {
    this.ensureCmdVelAdvertised()
    this.send({
      op: 'publish',
      topic: this.config.topics.cmdVel,
      msg: {
        linear: { x: Number(cmd.linear ?? 0), y: 0, z: 0 },
        angular: { x: 0, y: 0, z: Number(cmd.angular ?? 0) },
      },
    })
  }

  async publishNavGoal(goal: { x: number; y: number; yaw: number; frameId: string }): Promise<void> {
    this.ensureMoveBaseGoalAdvertised()
    if (this.config.publishSimpleGoal) {
      this.ensureMoveBaseSimpleGoalAdvertised()
    }
    const stamp = rosTime()
    const orientation = yawToQuaternion(goal.yaw)
    this.send({
      op: 'publish',
      topic: this.config.topics.moveBaseGoal,
      msg: {
        header: { seq: 0, stamp, frame_id: goal.frameId },
        goal_id: { stamp, id: `web-${Date.now()}` },
        goal: {
          target_pose: {
            header: { seq: 0, stamp, frame_id: goal.frameId },
            pose: {
              position: { x: Number(goal.x), y: Number(goal.y), z: 0 },
              orientation,
            },
          },
        },
      },
    })
    if (this.config.publishSimpleGoal) {
      this.send({
        op: 'publish',
        topic: this.config.topics.moveBaseSimpleGoal,
        msg: {
          header: { seq: 0, stamp, frame_id: goal.frameId },
          pose: {
            position: { x: Number(goal.x), y: Number(goal.y), z: 0 },
            orientation,
          },
        },
      })
    }
  }

  async publishOccupancyGrid(grid: { width: number; height: number; resolution: number; origin: { x: number; y: number }; data: number[] }): Promise<void> {
    this.send({
      op: 'publish',
      topic: this.config.topics.map,
      msg: {
        header: { seq: 0, stamp: rosTime(), frame_id: 'map' },
        info: {
          map_load_time: rosTime(),
          resolution: grid.resolution,
          width: grid.width,
          height: grid.height,
          origin: {
            position: { x: grid.origin.x, y: grid.origin.y, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
        data: grid.data,
      },
    })
  }

  async publishInitialPose(pose: { x: number; y: number; yaw: number; frameId: string }): Promise<void> {
    const stamp = rosTime()
    const orientation = yawToQuaternion(pose.yaw)
    this.send({
      op: 'publish',
      topic: '/initialpose',
      msg: {
        header: { seq: 0, stamp, frame_id: pose.frameId },
        pose: {
          pose: {
            position: { x: Number(pose.x), y: Number(pose.y), z: 0 },
            orientation,
          },
          covariance: new Array(36).fill(0) as number[],
        },
      },
    })
  }

  async publishSimpleGoal(goal: { x: number; y: number; yaw: number; frameId: string }): Promise<void> {
    console.log(`[rosAdapter] publishSimpleGoal → x=${goal.x} y=${goal.y} yaw=${goal.yaw} frame=${goal.frameId}`)
    this.ensureMoveBaseSimpleGoalAdvertised()
    const stamp = rosTime()
    const orientation = yawToQuaternion(goal.yaw)
    this.send({
      op: 'publish',
      topic: this.config.topics.moveBaseSimpleGoal,
      msg: {
        header: { seq: 0, stamp, frame_id: goal.frameId },
        pose: {
          position: { x: Number(goal.x), y: Number(goal.y), z: 0 },
          orientation,
        },
      },
    })
  }

  getStatus(): RosAdapterStatus {
    return {
      mode: 'rosbridge',
      rosbridgeUrl: this.config.rosbridgeUrl,
      connected: runtimeState.rosConnected,
      topics: this.config.topics,
      lastMapAt: runtimeState.lastMapAt,
      lastTfAt: runtimeState.lastTfAt,
      lastLidarAt: runtimeState.lastLidarAt,
    }
  }

  private connect() {
    this.cmdVelAdvertised = false
    this.virtualObstaclesAdvertised = false
    this.moveBaseGoalAdvertised = false
    this.moveBaseSimpleGoalAdvertised = false
    const ws = this.wsFactory(this.config.rosbridgeUrl)
    this.socket = ws

    ws.on('open', () => {
      if (this.socket !== ws) return
      runtimeState.rosConnected = true
      this.subscribeTopics()
      this.ensureCmdVelAdvertised()
      this.ensureVirtualObstaclesAdvertised()
      this.ensureMoveBaseGoalAdvertised()
      if (this.config.publishSimpleGoal) {
        this.ensureMoveBaseSimpleGoalAdvertised()
      }
    })

    ws.on('message', (raw) => {
      this.handleMessage(String(raw))
    })

    ws.on('close', () => {
      if (this.socket !== ws) return
      runtimeState.rosConnected = false
      this.socket = null
      if (this.shouldReconnect) {
        this.scheduleReconnect()
      }
    })

    ws.on('error', () => {
      runtimeState.rosConnected = false
    })
  }

  private scheduleReconnect() {
    if (this.reconnectTimer) return
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null
      if (this.shouldReconnect) {
        this.connect()
      }
    }, 1500)
  }

  private subscribeTopics() {
    // /map is NOT subscribed via rosbridge — large maps (42M cells) choke rosbridge's
    // JSON serializer (>200MB JSON, 1.7GB RAM). The gateway reads PGM from disk instead.
    const topics = [
      { topic: this.config.topics.tf, type: 'tf2_msgs/TFMessage' },
      this.config.topics.lidar ? { topic: this.config.topics.lidar, type: 'sensor_msgs/PointCloud2' } : null,
      this.config.topics.globalPlan ? { topic: this.config.topics.globalPlan, type: 'nav_msgs/Path' } : null,
      this.config.topics.localPlan ? { topic: this.config.topics.localPlan, type: 'nav_msgs/Path' } : null,
      this.config.topics.voxelGrid ? { topic: this.config.topics.voxelGrid, type: 'sensor_msgs/PointCloud2' } : null,
    ].filter(Boolean)
    console.log('[rosAdapter] subscribing to topics:', topics.map((t) => t!.topic))
    for (const t of topics) {
      if (t) this.send({ op: 'subscribe', topic: t.topic, type: t.type })
    }
  }

  private ensureCmdVelAdvertised() {
    if (this.cmdVelAdvertised) return
    this.send({ op: 'advertise', topic: this.config.topics.cmdVel, type: 'geometry_msgs/Twist' })
    this.cmdVelAdvertised = true
  }

  private ensureVirtualObstaclesAdvertised() {
    if (this.virtualObstaclesAdvertised) return
    this.send({
      op: 'advertise',
      topic: this.config.topics.virtualObstacles,
      type: this.config.virtualObstaclesType,
    })
    this.virtualObstaclesAdvertised = true
  }

  private ensureMoveBaseGoalAdvertised() {
    if (this.moveBaseGoalAdvertised) return
    this.send({ op: 'advertise', topic: this.config.topics.moveBaseGoal, type: 'move_base_msgs/MoveBaseActionGoal' })
    this.moveBaseGoalAdvertised = true
  }

  private ensureMoveBaseSimpleGoalAdvertised() {
    if (this.moveBaseSimpleGoalAdvertised) return
    this.send({ op: 'advertise', topic: this.config.topics.moveBaseSimpleGoal, type: 'geometry_msgs/PoseStamped' })
    this.moveBaseSimpleGoalAdvertised = true
  }

  private send(payload: Record<string, unknown>) {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      console.warn(`[rosAdapter] send dropped — socket state: ${this.socket?.readyState ?? 'null'}`, JSON.stringify(payload).slice(0, 200))
      return
    }
    console.log(`[rosAdapter] send → ${payload.op} ${payload.topic}`)
    this.socket.send(JSON.stringify(payload))
  }

  private handleMessage(raw: string) {
    let data: unknown
    try {
      data = JSON.parse(raw)
    } catch {
      return
    }
    const packet = data as { op?: string; topic?: string; msg?: unknown }
    if (packet.op !== 'publish' || !packet.topic) {
      return
    }

    if (packet.topic === this.config.topics.map) {
      const msg = packet.msg as {
        info?: { width?: number; height?: number; resolution?: number; origin?: { position?: { x?: number; y?: number } } }
        data?: number[]
      }
      const width = Number(msg.info?.width ?? 0)
      const height = Number(msg.info?.height ?? 0)
      if (width > 0 && height > 0 && Array.isArray(msg.data)) {
        const cells = width * height
        // Sanitize occupancy data: rosbridge may serialize int8[] as uint8[] (e.g. -1 → 255)
        // Detect by checking for values > 100; if found, re-interpret as signed bytes
        let dataArr = msg.data.map((v) => Number(v ?? 0))
        const maxVal = dataArr.reduce((a, b) => Math.max(a, b), -128)
        if (maxVal > 100) {
          console.warn(`[rosAdapter] Occupancy data contains values >100 (max=${maxVal}), normalizing uint8→int8`)
          dataArr = dataArr.map((v) => v > 127 ? v - 256 : v)
        }
        // Clamp to valid occupancy range
        for (let i = 0; i < dataArr.length; i++) {
          const v = dataArr[i]!
          if (v < -1) dataArr[i] = -1
          else if (v > 100) dataArr[i] = 100
        }
        const pgmDir = path.resolve(process.cwd(), 'data', 'maps')
        const isLarge = cells > LARGE_MAP_CELL_THRESHOLD

        runtimeState.map = {
          width,
          height,
          resolution: Number(msg.info?.resolution ?? runtimeState.map.resolution),
          origin: {
            x: Number(msg.info?.origin?.position?.x ?? 0),
            y: Number(msg.info?.origin?.position?.y ?? 0),
          },
          // Write PNG (for display) + PGM (for MapEditor editing)
          mapUrl: writeMapFiles(width, height, dataArr, pgmDir),
          // For large maps, discard raw data to save memory (~336MB for 42M cells)
          data: isLarge ? [] : dataArr,
        }
        runtimeState.lastMapAt = nowIso()
        runtimeState.mapUpdatedAt = runtimeState.lastMapAt
        if (isLarge) {
          console.log(`[rosAdapter] Large map received (${width}x${height}, ${(cells/1e6).toFixed(1)}M cells) → PGM only, raw data discarded`)
        }
      }
    } else if (packet.topic === this.config.topics.tf) {
      const msg = packet.msg as { transforms?: Array<{ child_frame_id?: string; header?: { frame_id?: string }; transform?: { translation?: { x?: number; y?: number }; rotation?: { x?: number; y?: number; z?: number; w?: number } } }> }
      const transforms = Array.isArray(msg.transforms) ? msg.transforms : []
      const preferred = transforms.find((t) => String(t.child_frame_id ?? '').includes('base')) ?? transforms[0]
      if (preferred?.transform?.translation) {
        runtimeState.tfPose = {
          x: Number(preferred.transform.translation.x ?? 0),
          y: Number(preferred.transform.translation.y ?? 0),
          yaw: quaternionToYaw(preferred.transform.rotation ?? {}),
          frame: String(preferred.header?.frame_id ?? runtimeState.tfPose.frame),
        }
        runtimeState.lastTfAt = nowIso()
      }
    } else if (packet.topic === this.config.topics.lidar) {
      runtimeState.lidar = parsePointCloud2(packet.msg)
      runtimeState.lastLidarAt = nowIso()
    } else if (packet.topic === this.config.topics.globalPlan) {
      const pts = parsePath(packet.msg)
      if (pts.length > 0) console.log(`[rosAdapter] globalPlan ← ${pts.length} pts, first=(${pts[0]?.x?.toFixed(2)},${pts[0]?.y?.toFixed(2)})`)
      runtimeState.globalPlan = pts
      runtimeState.lastGlobalPlanAt = nowIso()
    } else if (packet.topic === this.config.topics.localPlan) {
      const pts = parsePath(packet.msg)
      if (pts.length > 0) console.log(`[rosAdapter] localPlan ← ${pts.length} pts`)
      runtimeState.localPlan = pts
      runtimeState.lastLocalPlanAt = nowIso()
    } else if (packet.topic === this.config.topics.voxelGrid) {
      runtimeState.voxelGrid = parsePointCloud2(packet.msg)
      runtimeState.lastVoxelAt = nowIso()
    }

    runtimeState.lastSnapshotAt = nowIso()
  }
}

export function createRosAdapterFromEnv(): RosAdapter {
  const config = readRosConfig()
  if (config.mode === 'rosbridge') {
    return new RosbridgeAdapter(config)
  }
  return new MockRosAdapter(config)
}
