import cors from 'cors'
import dotenv from 'dotenv'
import express from 'express'
import { rateLimit, ipKeyGenerator } from 'express-rate-limit'
import multer from 'multer'
import bcrypt from 'bcryptjs'
import { createServer } from 'node:http'
import { mkdirSync, copyFileSync, existsSync, writeFileSync, readdirSync, readFileSync, unlinkSync, rmdirSync, renameSync, statSync, createReadStream } from 'node:fs'
import { freemem, totalmem, loadavg } from 'node:os'
import { deflateSync } from 'node:zlib'
import { spawn, exec, execFile, execSync, type ChildProcess } from 'node:child_process'
import path from 'node:path'
import { WebSocketServer } from 'ws'
import { connect, type Channel } from 'amqplib'
import db from './db.js'
import { getUserById, requireAuth, requireRole, signToken, type AuthedRequest } from './auth.js'
import { newId, runtimeState, updateMockPoseAndLidar, LARGE_MAP_CELL_THRESHOLD } from './state.js'
import { createRosAdapterFromEnv } from './rosAdapter.js'

dotenv.config()
const rosAdapter = createRosAdapterFromEnv()

// ====== MQ 发布 ======
const MQ_HOST = process.env.MQ_HOST || '127.0.0.1'
const MQ_PORT = Number(process.env.MQ_PORT || '5672')
const MQ_USER = process.env.MQ_USER || 'nav'
const MQ_PASS = process.env.MQ_PASS || 'nav123'
const MQ_VHOST = process.env.MQ_VHOST || '/'
const MQ_EXCHANGE = process.env.MQ_EXCHANGE || 'nav.exchange'
const MQ_CLIENT_ID = process.env.MQ_CLIENT_ID || 'B42D1000P3499GGW' //默认: robot-001 
let mqChannel: Channel | null = null
let mqConn: any = null
let lastMqPoseAt = 0   // MQ 位姿最后更新时间戳

async function mqConnect(): Promise<Channel | null> {
  try {
    if (mqChannel && mqConn) return mqChannel
    mqConn = await connect({
      hostname: MQ_HOST, port: MQ_PORT,
      username: MQ_USER, password: MQ_PASS,
      vhost: MQ_VHOST, heartbeat: 30,
    })
    mqChannel = await mqConn.createChannel()
    await mqChannel!.assertExchange(MQ_EXCHANGE, 'topic', { durable: true })

    // ---- 订阅 MQ 状态 & 位姿 & 路线（mq_adapter → Web） ----
    const q = await mqChannel!.assertQueue('', { exclusive: true, autoDelete: true })
    await mqChannel!.bindQueue(q.queue, MQ_EXCHANGE, `nav.${MQ_CLIENT_ID}.status`)
    await mqChannel!.bindQueue(q.queue, MQ_EXCHANGE, `nav.${MQ_CLIENT_ID}.pose`)
    await mqChannel!.bindQueue(q.queue, MQ_EXCHANGE, `nav.${MQ_CLIENT_ID}.route`)
    await mqChannel!.bindQueue(q.queue, MQ_EXCHANGE, `nav.${MQ_CLIENT_ID}.nav_points`)
    await mqChannel!.consume(q.queue, (msg) => {
      if (!msg) return
      try {
        const data = JSON.parse(msg.content.toString())
        const hdr = data.header || {}
        const body = data.body || {}
        if (hdr.msg_type === 'nav_status') {
          const ns = body.nav_state as string
          console.log(`[gateway] MQ ← status: ${ns}`)
          const map: Record<string, string> = { idle: 'idle', running: 'running', paused: 'paused', completed: 'stopped', failed: 'stopped', cancelled: 'stopped', '就绪': 'running', '运动中': 'running', '阻塞': 'running', '到达': 'stopped' }
          const mapped = map[ns] || ns
          // gateway 是状态源，MQ status 仅作外部遥测回显，不覆盖本地主动设置的状态
          const localActive = navProcess !== null || taskProcess !== null
          if (!localActive && runtimeState.navStatus !== mapped) {
            runtimeState.navStatus = mapped as 'idle' | 'running' | 'paused' | 'stopped'
            broadcast('nav_status', mapped)
            console.log(`[gateway] broadcast nav_status: ${mapped}`)
          }
          // 根据 current_cmd 同步 navMode
          const cmd = body.current_cmd as string
          if (cmd === 'nav_multi' && runtimeState.navMode !== 'multi') {
            runtimeState.navMode = 'multi'
          } else if (cmd === 'nav_single' && runtimeState.navMode !== 'single') {
            runtimeState.navMode = 'single'
          } else if (!cmd && mapped === 'stopped') {
            runtimeState.navMode = 'none'
          }
        } else if (hdr.msg_type === 'nav_pose') {
          runtimeState.tfPose = {
            x: body.position?.x ?? runtimeState.tfPose.x,
            y: body.position?.y ?? runtimeState.tfPose.y,
            yaw: body.orientation?.yaw ?? runtimeState.tfPose.yaw,
            frame: body.frame_id || 'camera_init',
          }
          lastMqPoseAt = Date.now()
        } else if (hdr.msg_type === 'nav_points') {
          const points = (body.points || []) as Array<{id: string; x: number; y: number; yaw: number}>
          if (points.length > 0) {
            // 同步写入 TASK_DATA 文件 + broadcast
            mkdirSync(TASK_DATA_DIR, { recursive: true })
            for (const f of readdirSync(TASK_DATA_DIR)) {
              if (f.endsWith('.json')) unlinkSync(path.join(TASK_DATA_DIR, f))
            }
            for (let i = 0; i < points.length; i++) {
              const p = points[i]
              writeNavPointFile(p.id || `wp-${i}`, p.id || `wp-${i}`, p.x, p.y, p.yaw ?? 0, i)
            }
            broadcastNavPointsFromFiles()
          }
        } else if (hdr.msg_type === 'nav_route') {
          const pts: Array<{x: number; y: number}> = body.path || []
          if (pts.length >= 2) {
            runtimeState.globalPlan = pts
          }
        }
      } catch { /* 忽略解析错误 */ }
    }, { noAck: true })

    // 重连处理
    mqConn.on('close', () => { mqChannel = null; mqConn = null })
    mqConn.on('error', () => { mqChannel = null; mqConn = null })
    console.log('[gateway] MQ connected + subscribed status/pose')
    return mqChannel
  } catch (e) {
    console.warn('[gateway] MQ connect failed:', e)
    mqChannel = null; mqConn = null
    return null
  }
}

async function mqPublish(suffix: string, msgType: string, body: Record<string, unknown>): Promise<boolean> {
  const ch = await mqConnect()
  if (!ch) return false
  try {
    const mid = crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}-${Math.random().toString(36).slice(2, 10)}`
    const msg = {
      header: {
        msg_type: msgType,
      },
      body: { ...body, ref_msg_id: mid },
    }
    const routingKey = `nav.${MQ_CLIENT_ID}.${suffix}`
    ch.publish(MQ_EXCHANGE, routingKey, Buffer.from(JSON.stringify(msg)))
    console.log(`[gateway] MQ → ${routingKey}`)
    return true
  } catch (e) {
    console.error('[gateway] MQ publish error:', e)
    return false
  }
}
const app = express()
const server = createServer(app)
const wss = new WebSocketServer({ server, path: '/ws' })
const upload = multer({ dest: path.resolve(process.cwd(), 'uploads') })
const PORT = Number(process.env.GATEWAY_PORT ?? 8080)
const mapDir = path.resolve(process.cwd(), 'data', 'maps')
const navGoalRateWindowMs = Number(process.env.NAV_GOAL_RATE_LIMIT_WINDOW_MS ?? 5000)
const navGoalRateLimitCount = Number(process.env.NAV_GOAL_RATE_LIMIT_COUNT ?? 20)
const SLAM_SCRIPT = process.env.SLAM_START_SCRIPT || '/home/robot/go2_nav/SC-FAST-LIO-main/start_slam.sh'
const NAV_SCRIPT  = process.env.NAV_START_SCRIPT  || '/home/robot/go2_nav/lite_cog/system/scripts/nav/start_nav.sh'
const TASK_SCRIPT = process.env.TASK_START_SCRIPT || '/home/robot/go2_nav/lite_cog/pipeline/src/pipeline_tracking/scripts/Task.py'
const LIDAR_SCRIPT = process.env.LIDAR_START_SCRIPT || '/home/robot/go2_nav/lite_cog/system/scripts/lidar/start_lidar.sh'
const MOTION_SCRIPT = process.env.MOTION_START_SCRIPT || '/home/robot/go2_nav/go2_SDK_ws/go2_cmd_vel.sh'
mkdirSync(mapDir, { recursive: true })

// ── PNG generation from PGM (零依赖, 纯 zlib) ──
function crc32Table(): Uint32Array {
  const t = new Uint32Array(256)
  for (let n = 0; n < 256; n++) {
    let c = n
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1
    t[n] = c
  }
  return t
}
const CRC_TABLE = crc32Table()
function crc32(buf: Buffer): number {
  let c = 0xffffffff
  for (let i = 0; i < buf.length; i++) c = CRC_TABLE[(c ^ buf[i]!) & 0xff]! ^ (c >>> 8)
  return (c ^ 0xffffffff) >>> 0
}
function pngChunk(type: string, data: Buffer): Buffer {
  const typeData = Buffer.concat([Buffer.from(type, 'ascii'), data])
  const len = Buffer.alloc(4); len.writeUInt32BE(data.length, 0)
  const crcBuf = Buffer.alloc(4); crcBuf.writeUInt32BE(crc32(typeData), 0)
  return Buffer.concat([len, typeData, crcBuf])
}

/** Read PGM file → write PNG+PGM to static/maps/ → update runtimeState.map */
function regenerateMapPng(pgmPath: string): void {
  try {
    const buf = readFileSync(pgmPath)
    // Parse PGM header — handle comment lines (#) by skipping them
    // PGM format: P5\n [#comments\n]* <width> <height>\n 255\n <pixel data>
    let pos = 0
    const readLine = (): string => {
      const start = pos
      while (pos < buf.length && buf[pos] !== 10) pos++
      const line = buf.subarray(start, pos).toString()
      pos++ // skip \n
      return line
    }
    // Magic number
    const magic = readLine()
    if (magic !== 'P5') { console.error('[png] Invalid PGM header (not P5)'); return }
    // Skip comment lines
    let dimLine = readLine()
    while (dimLine.startsWith('#') || dimLine.trim() === '') {
      dimLine = readLine()
    }
    const dims = dimLine.trim().split(/\s+/)
    const width = parseInt(dims[0]!), height = parseInt(dims[1]!)
    // Read maxval line (may have preceding comments, but standard is just "255")
    let maxvalLine = readLine()
    while (maxvalLine.startsWith('#') || maxvalLine.trim() === '') {
      maxvalLine = readLine()
    }
    const hdrEnd = pos
    const imgData = buf.subarray(hdrEnd)

    // Read YAML for origin + resolution (same directory, .yaml extension)
    const yamlPath = pgmPath.replace(/\.pgm$/i, '.yaml')
    let resolution = 0.05, ox = 0, oy = 0
    if (existsSync(yamlPath)) {
      try {
        const yaml = readFileSync(yamlPath, 'utf-8')
        const resMatch = yaml.match(/resolution:\s*([\d.]+)/)
        const origMatch = yaml.match(/origin:\s*\[([-\d.]+),\s*([-\d.]+)/)
        if (resMatch) resolution = parseFloat(resMatch[1]!)
        if (origMatch) { ox = parseFloat(origMatch[1]!); oy = parseFloat(origMatch[2]!) }
        console.log(`[png] YAML origin: [${ox}, ${oy}], resolution: ${resolution}`)
      } catch { /* YAML read error, use defaults */ }
    }

    // Copy PGM to static dir for MapEditor editing
    const pgmOut = path.join(staticMapsDir, 'live_map.pgm')
    copyFileSync(pgmPath, pgmOut)

    // Build PNG raw rows (filter byte 0 + width grayscale bytes per row)
    const rawRows = Buffer.alloc((width + 1) * height)
    for (let y = 0; y < height; y++) {
      rawRows[y * (width + 1)] = 0
      for (let x = 0; x < width; x++) {
        rawRows[y * (width + 1) + 1 + x] = imgData[y * width + x]!
      }
    }
    const sig = Buffer.from([137, 80, 78, 71, 13, 10, 26, 10])
    const ihdrData = Buffer.alloc(13)
    ihdrData.writeUInt32BE(width, 0); ihdrData.writeUInt32BE(height, 4)
    ihdrData[8] = 8; ihdrData[9] = 0; ihdrData[10] = 0; ihdrData[11] = 0; ihdrData[12] = 0
    const png = Buffer.concat([
      sig, pngChunk('IHDR', ihdrData),
      pngChunk('IDAT', deflateSync(rawRows)),
      pngChunk('IEND', Buffer.alloc(0)),
    ])
    const pngOut = path.join(staticMapsDir, 'live_map.png')
    writeFileSync(pngOut, png)

    const cells = width * height
    const isLarge = cells > LARGE_MAP_CELL_THRESHOLD
    // Convert PGM pixel values → ROS occupancy values (same thresholds as frontend loadPngMapTexture)
    //   PGM 0 (black)    → 100 (occupied) → displays as black  ✓
    //   PGM 254 (white)  → 0   (free)     → displays as white  ✓
    //   PGM 205 (gray)   → -1  (unknown)  → displays transparent ✓
    // ALSO Y-flip: PGM row 0 (top) → ROS row (height-1) (top)
    //   PGM rows are top→bottom; ROS OccupancyGrid rows are bottom→top (origin at bottom-left)
    let occData: number[] = []
    if (!isLarge) {
      occData = new Array(imgData.length)
      for (let y = 0; y < height; y++) {
        const srcRow = y * width                    // PGM row y (top of image)
        const dstRow = (height - 1 - y) * width     // ROS row (bottom of map = row 0)
        for (let x = 0; x < width; x++) {
          const v = imgData[srcRow + x]!
          if (v <= 1) occData[dstRow + x] = 100       // near-black → occupied
          else if (v >= 250) occData[dstRow + x] = 0   // near-white → free
          else occData[dstRow + x] = -1                // mid-gray → unknown
        }
      }
    }
    runtimeState.map = {
      width, height, resolution,
      origin: { x: ox, y: oy },
      mapUrl: `/static/maps/live_map.png?t=${Date.now()}`,
      data: occData,
    }
    runtimeState.mapUpdatedAt = new Date().toISOString()
    console.log(`[png] live_map: ${width}x${height} → PNG ${(png.length/1024).toFixed(0)}KB + PGM ${(imgData.length/1024/1024).toFixed(1)}MB (${isLarge ? 'large' : 'small'}) ${isLarge ? '' : '+data'}`)
  } catch (e) {
    console.error('[png] Failed:', e)
  }
}

/** 递归搜索目录下所有 PCD 文件，排除衍生文件 */
function findAllPcds(dir: string): Array<{ path: string; mtimeMs: number }> {
  const results: Array<{ path: string; mtimeMs: number }> = []
  function walk(currentDir: string) {
    let entries: string[]
    try { entries = readdirSync(currentDir) } catch { return }
    for (const name of entries) {
      const fp = path.join(currentDir, name)
      let st
      try { st = statSync(fp) } catch { continue }
      if (st.isDirectory()) {
        // Skip intermediate scan data folder
        const base = name.toLowerCase()
        if (base === 'scans' || base === 'active' || base === '__pycache__' || base === '.claude') continue
        walk(fp)
        continue
      }
      if (!name.endsWith('.pcd')) continue
      const base = name.toLowerCase()
      if (base.includes('leveled') || base.includes('colored') || base.includes('_z_range') || base === 'current.pcd') continue
      results.push({ path: fp, mtimeMs: st.mtimeMs })
    }
  }
  walk(dir)
  return results
}

// 进程管理
let slamProcess: ChildProcess | null = null
let navProcess: ChildProcess | null = null
let taskProcess: ChildProcess | null = null
let lidarProcess: ChildProcess | null = null
let motionProcess: ChildProcess | null = null
let taskRestarting = false
let navGeneration = 0  // 防止旧进程 exit 回调覆盖新状态

function killProcess(proc: ChildProcess | null, label: string): boolean {
  if (!proc || proc.killed) return false
  try {
    const pid = proc.pid
    proc.kill('SIGTERM')
    if (pid) { try { process.kill(-pid, 'SIGTERM') } catch {} }
    setTimeout(() => {
      if (pid) { try { process.kill(-pid, 'SIGKILL') } catch {} }
      try { if (!proc.killed) proc.kill('SIGKILL') } catch {}
    }, 500)
    console.log(`[gateway] killed ${label}`)
    return true
  } catch { return false }
}

app.use(cors())
app.use(express.json({ limit: '256mb' }))

// Serve production frontend build (no vite dev server needed)
const distDir = path.resolve(process.cwd(), '..', 'dist')
if (existsSync(distDir)) {
  app.use(express.static(distDir, { maxAge: 0 }))
  console.log('[gateway] Serving frontend from', distDir)
}

// Serve live PGM/PNG maps for frontend to load as images
const staticMapsDir = path.resolve(process.cwd(), 'data', 'maps')
mkdirSync(staticMapsDir, { recursive: true })
app.use('/static/maps', express.static(staticMapsDir, { maxAge: 0, setHeaders: (res) => {
  res.setHeader('Cache-Control', 'public, max-age=10')
}}))

function readNavPointsFromFiles(): Array<{id: string; name: string; x: number; y: number; yaw: number; map_id: string | null; created_at: string}> {
  const items: Array<{order: number; point: {id: string; name: string; x: number; y: number; yaw: number; map_id: string | null; created_at: string}}> = []
  try {
    const files = readdirSync(TASK_DATA_DIR).filter(f => f.endsWith('.json'))
    for (const f of files) {
      try {
        const raw = JSON.parse(readFileSync(path.join(TASK_DATA_DIR, f), 'utf-8'))
        const rp = raw.robot_pose || {}
        const id = f.replace('.json', '')
        items.push({
          order: typeof raw.order === 'number' ? raw.order : 0,
          point: {
            id,
            name: typeof raw.name === 'string' ? raw.name : id,
            x: rp.pos_x ?? 0, y: rp.pos_y ?? 0,
            yaw: Math.atan2(rp.ori_z || 0, rp.ori_w || 1) * 2,
            map_id: null, created_at: new Date().toISOString(),
          },
        })
      } catch { /* skip bad files */ }
    }
    items.sort((a, b) => a.order - b.order)
  } catch { /* dir not exist */ }
  return items.map(it => it.point)
}

function snapshot() {
  const navPoints = readNavPointsFromFiles()
  const obstacles = parseObstacleRows(db.prepare('SELECT * FROM virtual_obstacles').all())
  const maps = db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all()
  return {
    runtime: runtimeState,
    navPoints,
    virtualObstacles: obstacles,
    maps,
  }
}

function parseObstacleRows(rows: unknown[]) {
  return rows.map((row) => {
    const o = row as { data: string }
    return { ...(row as Record<string, unknown>), data: JSON.parse(String(o.data)) }
  })
}

function broadcast(event: string, payload: unknown) {
  const data = JSON.stringify({ event, payload })
  wss.clients.forEach((client) => {
    if (client.readyState === client.OPEN) {
      client.send(data)
    }
  })
}

// Health check
app.get('/api/health', (_req, res) => {
  const rosStatus = rosAdapter.getStatus()
  const [l1, l5, l15] = loadavg()
  res.json({
    ok: true,
    rosConnected: runtimeState.rosConnected,
    mappingPauseSupported: runtimeState.mappingPauseSupported,
    ros: {
      mode: rosStatus.mode,
      rosbridgeUrl: rosStatus.rosbridgeUrl,
      connected: rosStatus.connected,
      topics: rosStatus.topics,
      lastReceivedAt: {
        map: rosStatus.lastMapAt,
        tf: rosStatus.lastTfAt,
        lidar: rosStatus.lastLidarAt,
      },
    },
    system: {
      memFree: freemem(),
      memTotal: totalmem(),
      memUsedPercent: Math.round((1 - freemem() / totalmem()) * 100),
      heapUsedMB: Math.round(process.memoryUsage().heapUsed / 1024 / 1024),
      load1m: Math.round(l1! * 100) / 100,
      load5m: Math.round(l5! * 100) / 100,
      load15m: Math.round(l15! * 100) / 100,
    },
  })
})

// Snapshot
app.get('/api/snapshot', requireAuth, (_req, res) => {
  res.json(snapshot())
})

// Auth
app.post('/api/auth/login', (req, res) => {
  const { username, password } = req.body ?? {}
  const user = db.prepare('SELECT * FROM users WHERE username = ?').get(username) as
    | { id: string; username: string; role: 'admin' | 'operator'; password_hash: string }
    | undefined
  if (!user || !bcrypt.compareSync(String(password ?? ''), user.password_hash)) {
    res.status(401).json({ error: 'Invalid credentials' })
    return
  }
  res.json({
    token: signToken({ sub: user.id, username: user.username, role: user.role }),
    user: { id: user.id, username: user.username, role: user.role },
  })
})

app.get('/api/auth/me', requireAuth, (req: AuthedRequest, res) => {
  const user = req.user ? getUserById(req.user.sub) : undefined
  res.json({ user })
})

// Users (admin only)
app.get('/api/users', requireAuth, requireRole(['admin']), (_req, res) => {
  const users = db.prepare('SELECT id, username, role, created_at FROM users ORDER BY created_at DESC').all()
  res.json({ users })
})

app.post('/api/users', requireAuth, requireRole(['admin']), (req, res) => {
  const { username, password, role } = req.body ?? {}
  if (!username || !password || !['admin', 'operator'].includes(role)) {
    res.status(400).json({ error: 'Invalid payload' })
    return
  }
  try {
    db.prepare('INSERT INTO users (id, username, role, password_hash, created_at) VALUES (?, ?, ?, ?, ?)').run(
      newId(),
      String(username),
      role,
      bcrypt.hashSync(String(password), 10),
      new Date().toISOString(),
    )
    res.status(201).json({ ok: true })
  } catch {
    res.status(409).json({ error: 'Username already exists' })
  }
})

// Mapping control
app.post('/api/mapping/start', requireAuth, async (_req, res) => {
  if (slamProcess && !slamProcess.killed) {
    res.json({ ok: true, message: '建图已在运行' })
    return
  }
  try {
    // 清空地图 + 杀旧 map_server
    runtimeState.map = { width: 0, height: 0, resolution: 0.05, origin: { x: 0, y: 0 }, data: [], mapUrl: null }
    exec('pkill -f "[m]ap_server" 2>/dev/null; pkill -f "[M]apServer" 2>/dev/null', () => {})
    slamProcess = spawn('bash', [SLAM_SCRIPT], {
      detached: true,
      stdio: 'ignore',
    })
    slamProcess.on('exit', (code) => {
      console.log(`[gateway] SLAM exited with code ${code}`)
      slamProcess = null
      runtimeState.mappingStatus = 'stopped'
      broadcast('mapping_status', 'stopped')
    })
    runtimeState.mappingStatus = 'running'
    broadcast('mapping_status', 'running')
    console.log(`[gateway] SLAM started: ${SLAM_SCRIPT}`)
    res.json({ ok: true })
  } catch (e) {
    res.status(500).json({ error: String(e) })
  }
})

app.post('/api/mapping/pause', requireAuth, async (_req, res) => {
  res.status(409).json({
    ok: false,
    code: 'MAPPING_PAUSE_NOT_SUPPORTED',
    message: '当前建图程序不支持真实 pause。',
  })
})

app.post('/api/mapping/stop', requireAuth, async (req, res) => {
  const shouldSave = req.body?.save !== false
  if (shouldSave) {
    // Graceful shutdown: send SIGTERM only (no SIGKILL), let SLAM save PCD
    if (slamProcess && !slamProcess.killed) {
      try {
        const pid = slamProcess.pid
        slamProcess.kill('SIGTERM')
        if (pid) { try { process.kill(-pid, 'SIGTERM') } catch {} }
        console.log('[gateway] sent SIGTERM to SLAM for graceful PCD save')
      } catch { }
    }
    // Also SIGTERM any remaining fastlio_mapping processes
    exec('pkill -SIGTERM -f "[f]astlio_mapping" 2>/dev/null', () => {})
  } else {
    killProcess(slamProcess, 'SLAM')
    exec('pkill -f "[m]apping_mid360" 2>/dev/null; pkill -f "[f]astlio_mapping" 2>/dev/null; pkill -f "[g]ridmap" 2>/dev/null; pkill -f "[s]ave_map" 2>/dev/null; pkill -f "[l]ivox_ros_driver2" 2>/dev/null; pkill -f "[m]sg_MID360" 2>/dev/null; pkill -f "[r]viz" 2>/dev/null', () => {})
  }
  slamProcess = null
  runtimeState.mappingStatus = 'stopped'
  broadcast('mapping_status', 'stopped')
  res.json({ ok: true })

  // PCD → 2D grid map conversion after stopping (only if save=true)
  if (shouldSave) {
    const stopHook = process.env.MAP_STOP_HOOK
    if (stopHook) {
      setTimeout(() => {
        try {
          console.log('[gateway] Running MAP_STOP_HOOK:', stopHook)
          execSync(stopHook, { stdio: 'inherit', timeout: 300_000 })
          console.log('[gateway] MAP_STOP_HOOK done')
          broadcast('maps', db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all())
          const latest = db.prepare('SELECT name, pcd_path FROM maps ORDER BY rowid DESC LIMIT 1').get() as { name: string; pcd_path: string } | undefined
          const pcdName = latest?.pcd_path ? path.basename(latest.pcd_path) : '未知'
          broadcast('mapping_complete', { name: latest?.name || 'unknown', pcdFile: pcdName })
        } catch (e: any) {
          console.error('[gateway] MAP_STOP_HOOK failed:', e.message)
        }
      }, 2000)
    }
  }
})

// Maps
app.post('/api/maps/save', requireAuth, async (req, res) => {
  const name = String(req.body?.name ?? '').trim()
  if (!name) {
    res.status(400).json({ error: 'Map name required' })
    return
  }

  /** 将栅格数据写入指定地图文件夹 */
  function writeMapFiles(
    grid: { width: number; height: number; resolution: number; origin: { x: number; y: number }; data: number[] },
    mapDir: string,
  ): { pgmPath: string; yamlPath: string } {
    const mapFolder = path.join(mapDir, name)
    mkdirSync(mapFolder, { recursive: true })
    const pgmPath = path.join(mapFolder, `${name}.pgm`)
    const yamlPath = path.join(mapFolder, `${name}.yaml`)
    const resolution = grid.resolution || 0.05
    const ox = grid.origin?.x ?? 0
    const oy = grid.origin?.y ?? 0
    const w = grid.width
    const h = grid.height
    // PGM binary
    const header = Buffer.from(`P5\n${w} ${h}\n255\n`)
    const img = Buffer.alloc(w * h)
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const srcIdx = (h - 1 - y) * w + x
        const v = grid.data[srcIdx] ?? -1
        img[y * w + x] = v === 100 ? 0 : v === 0 ? 254 : 205
      }
    }
    writeFileSync(pgmPath, Buffer.concat([header, img]))
    // YAML 中 image 使用相对路径，PGM 与 YAML 在同一文件夹
    writeFileSync(yamlPath, `image: ${name}.pgm\nresolution: ${resolution}\norigin: [${ox}, ${oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n`)
    return { pgmPath, yamlPath }
  }

  /** 尝试将 PCD 复制到地图文件夹。如果指定了 pcdSource 且文件存在则直接使用。
   *  根目录下的 PCD（如 scans_1.pcd）用移动代替复制，避免与文件夹内的 PCD 重复。 */
  function copyPcdToMapFolder(mapDir: string, pcdSource?: string | null): string {
    const mapFolder = path.join(mapDir, name)
    mkdirSync(mapFolder, { recursive: true })
    const pcdDest = path.join(mapFolder, `${name}.pcd`)

    // 根目录下的 PCD 用移动（rename），避免残留重复文件；其他路径用复制
    function moveOrCopy(src: string, dst: string, label: string) {
      if (path.dirname(src) === mapDir) {
        renameSync(src, dst)
        console.log(`[gateway] PCD moved (root cleanup): ${src} -> ${dst} (${label})`)
      } else {
        copyFileSync(src, dst)
        console.log(`[gateway] PCD copied: ${src} -> ${dst} (${label})`)
      }
    }

    // 0) 用户手动指定的 PCD 优先
    if (pcdSource && existsSync(pcdSource)) {
      moveOrCopy(pcdSource, pcdDest, 'user selection')
      return pcdDest
    }

    // 1) 从当前激活地图继承 PCD
    const activeMap = db.prepare('SELECT pcd_path FROM maps WHERE active = 1').get() as { pcd_path: string } | undefined
    if (activeMap?.pcd_path && existsSync(activeMap.pcd_path)) {
      copyFileSync(activeMap.pcd_path, pcdDest)
      console.log(`[gateway] PCD copied from active map: ${activeMap.pcd_path} -> ${pcdDest}`)
      return pcdDest
    }

    // 2) 递归搜索 mapDir 下所有 PCD（含子文件夹）
    const allPcds = findAllPcds(mapDir)
    if (allPcds.length === 0) return ''

    // 优先匹配：文件名与地图名相同的 PCD
    const nameMatch = allPcds.find(p => path.basename(p.path, '.pcd') === name)
    if (nameMatch) {
      moveOrCopy(nameMatch.path, pcdDest, 'name match')
      return pcdDest
    }

    // 取最新修改的 PCD
    allPcds.sort((a, b) => b.mtimeMs - a.mtimeMs)
    const latest = allPcds[0]
    moveOrCopy(latest.path, pcdDest, 'latest')
    return pcdDest
  }

  const mapDir = process.env.MAP_DIR || '/home/robot/go2_nav/lite_cog/system/map'
  const gridData = req.body?.data as { width: number; height: number; resolution: number; origin: { x: number; y: number }; data: number[] } | undefined

  let pgmPath = ''
  let yamlPath = ''

  if (gridData && Array.isArray(gridData.data) && gridData.width > 0 && gridData.height > 0) {
    const result = writeMapFiles(gridData, mapDir)
    pgmPath = result.pgmPath
    yamlPath = result.yamlPath
  } else {
    // 没有 grid 数据，尝试用 runtimeState 中缓存的 /map 数据写文件
    const cachedMap = runtimeState.map
    if (cachedMap && cachedMap.data && cachedMap.data.length > 0 && cachedMap.width > 0) {
      const result = writeMapFiles(
        { width: cachedMap.width, height: cachedMap.height, resolution: cachedMap.resolution, origin: cachedMap.origin, data: cachedMap.data },
        mapDir,
      )
      pgmPath = result.pgmPath
      yamlPath = result.yamlPath
    } else if (cachedMap && cachedMap.width > 0 && existsSync(path.join(staticMapsDir, 'live_map.pgm'))) {
      // 大图：runtime 中 data 为空，直接用已生成的 PGM 文件
      const mapFolder = path.join(mapDir, name)
      mkdirSync(mapFolder, { recursive: true })
      pgmPath = path.join(mapFolder, `${name}.pgm`)
      yamlPath = path.join(mapFolder, `${name}.yaml`)
      copyFileSync(path.join(staticMapsDir, 'live_map.pgm'), pgmPath)
      writeFileSync(yamlPath, `image: ${name}.pgm\nresolution: ${cachedMap.resolution}\norigin: [${cachedMap.origin.x}, ${cachedMap.origin.y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n`)
    } else {
      res.status(400).json({ error: '/map 无数据。请先在编辑器中点「发布」将栅格推送到 /map 话题，再保存。' })
      return
    }
  }

  // 尝试复制 PCD 到地图文件夹（优先使用用户选择的）
  const userPcd = req.body?.pcd_path ? String(req.body.pcd_path).trim() : null
  const pcdPath = copyPcdToMapFolder(mapDir, userPcd || undefined)

  db.prepare('DELETE FROM maps WHERE name = ?').run(name)
  const id = newId()
  db.prepare('INSERT INTO maps (id, name, yaml_path, pcd_path, created_at, active) VALUES (?, ?, ?, ?, ?, 0)').run(id, name, yamlPath, pcdPath, new Date().toISOString())
  broadcast('maps', db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all())
  console.log(`[gateway] Saved map "${name}" -> ${path.join(mapDir, name)}/ (pgm+${pcdPath ? 'pcd' : 'no-pcd'})`)
  res.status(201).json({ id })
})

app.get('/api/maps', requireAuth, (_req, res) => {
  const maps = db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all()
  res.json({ maps })
})

// 列出所有可选的 PCD 文件
app.get('/api/maps/pcds', requireAuth, (_req, res) => {
  const mapDir = process.env.MAP_DIR || '/home/robot/go2_nav/lite_cog/system/map'
  const pcds = findAllPcds(mapDir).sort((a, b) => b.mtimeMs - a.mtimeMs)
  res.json({
    pcds: pcds.map(p => ({
      path: p.path,
      name: path.basename(p.path, '.pcd'),
      folder: path.basename(path.dirname(p.path)),
      mtimeMs: p.mtimeMs,
      mtime: new Date(p.mtimeMs).toISOString(),
    })),
  })
})

app.post('/api/maps/import', requireAuth, upload.single('mapFile'), (req, res) => {
  const file = req.file
  if (!file) {
    res.status(400).json({ error: 'mapFile is required' })
    return
  }
  const originalName = file.originalname || 'imported.pgm'
  if (!originalName.toLowerCase().endsWith('.pgm')) {
    res.status(400).json({ error: '只支持 .pgm 文件' })
    return
  }
  const mapName = originalName.replace(/\.pgm$/i, '')
  const mapDir = process.env.MAP_DIR || '/home/robot/go2_nav/lite_cog/system/map'
  // 为导入地图创建同名文件夹
  const mapFolder = path.join(mapDir, mapName)
  mkdirSync(mapFolder, { recursive: true })
  const pgmPath = path.join(mapFolder, `${mapName}.pgm`)
  const yamlPath = path.join(mapFolder, `${mapName}.yaml`)
  // PGM
  copyFileSync(file.path, pgmPath)
  // 自动生成 YAML
  const yamlContent = `image: ${mapName}.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
`
  writeFileSync(yamlPath, yamlContent)
  // PCD 复制逻辑：用户指定 > 激活地图继承 > 自动搜索
  // 根目录下的 PCD 用移动代替复制，避免残留重复文件
  const moveOrCopyToFolder = (src: string, dst: string) => {
    if (path.dirname(src) === mapDir) {
      renameSync(src, dst)
      console.log(`[gateway] Import PCD moved (root cleanup): ${src} -> ${dst}`)
    } else {
      copyFileSync(src, dst)
      console.log(`[gateway] Import PCD copied: ${src} -> ${dst}`)
    }
  }
  const userPcd = req.body?.pcd_path ? String(req.body.pcd_path).trim() : ''
  let pcdPath = ''
  if (userPcd && existsSync(userPcd)) {
    const pcdDest = path.join(mapFolder, `${mapName}.pcd`)
    moveOrCopyToFolder(userPcd, pcdDest)
    pcdPath = pcdDest
  } else {
    const activeMap = db.prepare('SELECT pcd_path FROM maps WHERE active = 1').get() as { pcd_path: string } | undefined
    if (activeMap?.pcd_path && existsSync(activeMap.pcd_path)) {
      const pcdDest = path.join(mapFolder, `${mapName}.pcd`)
      copyFileSync(activeMap.pcd_path, pcdDest)
      pcdPath = pcdDest
    } else {
      // 自动搜索
      const allPcds = findAllPcds(mapDir)
      if (allPcds.length > 0) {
        allPcds.sort((a, b) => b.mtimeMs - a.mtimeMs)
        const pcdDest = path.join(mapFolder, `${mapName}.pcd`)
        moveOrCopyToFolder(allPcds[0].path, pcdDest)
        pcdPath = pcdDest
      }
    }
  }
  db.prepare('DELETE FROM maps WHERE name = ?').run(mapName)
  db.prepare('INSERT INTO maps (id, name, yaml_path, pcd_path, created_at, active) VALUES (?, ?, ?, ?, ?, 0)').run(
    newId(),
    mapName,
    yamlPath,
    pcdPath,
    new Date().toISOString(),
  )
  broadcast('maps', db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all())
  res.status(201).json({ ok: true })
})

app.get('/api/maps/:id/export', requireAuth, (req, res) => {
  const row = db.prepare('SELECT * FROM maps WHERE id = ?').get(req.params.id) as { yaml_path: string; name: string } | undefined
  if (!row) {
    res.status(404).json({ error: 'Map not found' })
    return
  }
  const yamlAbs = row.yaml_path.startsWith('/') ? row.yaml_path : path.resolve(process.cwd(), row.yaml_path)
  const folder = path.dirname(yamlAbs)
  if (!existsSync(folder)) {
    res.status(404).json({ error: 'Map folder not found', path: folder })
    return
  }
  try {
    const tmpZip = `/tmp/map_export_${row.name}_${Date.now()}.zip`
    // 打包整个地图文件夹（包含 .yaml .pgm .pcd 等所有文件）
    execSync(`cd "${path.dirname(folder)}" && zip -0 -r "${tmpZip}" "${path.basename(folder)}"`, { timeout: 600_000 })
    // 用流式发送替代 res.download()，避免远程慢速下载时回调提前触发删文件
    const fileStat = statSync(tmpZip)
    const safeName = encodeURIComponent(row.name) + '.zip'
    res.writeHead(200, {
      'Content-Type': 'application/zip',
      'Content-Disposition': `attachment; filename*=UTF-8''${safeName}`,
      'Content-Length': fileStat.size,
    })
    const stream = createReadStream(tmpZip)
    stream.pipe(res)
    res.on('close', () => {
      try { unlinkSync(tmpZip) } catch {}
    })
  } catch (e: any) {
    console.error('[gateway] Map export failed:', e.message)
    res.status(500).json({ error: 'Export failed: ' + e.message })
  }
})

app.delete('/api/maps/:id', requireAuth, requireRole(['admin']), (req, res) => {
  const row = db.prepare('SELECT * FROM maps WHERE id = ?').get(req.params.id) as { name: string; yaml_path: string } | undefined
  db.prepare('DELETE FROM maps WHERE id = ?').run(req.params.id)
  // 清理地图文件夹
  if (row?.yaml_path) {
    const folder = path.dirname(row.yaml_path)
    try {
      if (existsSync(folder)) {
        // 删除文件夹内所有文件
        const files = readdirSync(folder)
        for (const f of files) {
          unlinkSync(path.join(folder, f))
        }
        rmdirSync(folder)
        console.log(`[gateway] Deleted map folder: ${folder}`)
      }
    } catch (e) { console.error('[gateway] cleanup folder error:', e) }
  }
  broadcast('maps', db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all())
  res.json({ ok: true })
})

// 更新某个地图的 PCD 关联
app.put('/api/maps/:id/pcd', requireAuth, (req, res) => {
  const { pcd_path } = req.body ?? {}
  if (typeof pcd_path !== 'string') {
    res.status(400).json({ error: 'pcd_path required' })
    return
  }
  db.prepare('UPDATE maps SET pcd_path = ? WHERE id = ?').run(pcd_path, req.params.id)
  broadcast('maps', db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all())
  res.json({ ok: true })
})

// 重命名地图（同时改文件和栅格关联）
app.put('/api/maps/:id/rename', requireAuth, (req, res) => {
  const newName = String(req.body?.name ?? '').trim()
  if (!newName) { res.status(400).json({ error: 'name required' }); return }
  const row = db.prepare('SELECT * FROM maps WHERE id = ?').get(req.params.id) as { name: string; yaml_path: string; pcd_path: string } | undefined
  if (!row) { res.status(404).json({ error: 'Map not found' }); return }
  const oldName = row.name
  const mapDir = process.env.MAP_DIR || '/home/robot/go2_nav/lite_cog/system/map'

  // 旧文件夹和新文件夹
  const oldFolder = path.dirname(row.yaml_path) // 例如 /home/.../map/oldName
  const newFolder = path.join(mapDir, newName)

  const newYaml = path.join(newFolder, `${newName}.yaml`)
  const newPgm = path.join(newFolder, `${newName}.pgm`)
  let newPcd = ''
  if (row.pcd_path) {
    newPcd = path.join(newFolder, `${newName}.pcd`)
  }

  try {
    // 如果旧文件夹存在，重命名整个文件夹（保留所有文件）
    if (existsSync(oldFolder)) {
      mkdirSync(path.dirname(newFolder), { recursive: true })
      // 先创建新文件夹（以防 renameSync 失败）
      mkdirSync(newFolder, { recursive: true })
      // 移动文件
      const oldYaml = path.join(oldFolder, `${oldName}.yaml`)
      const oldPgm = path.join(oldFolder, `${oldName}.pgm`)
      const oldPcd = path.join(oldFolder, `${oldName}.pcd`)
      if (existsSync(oldPgm)) { copyFileSync(oldPgm, newPgm); unlinkSync(oldPgm) }
      if (existsSync(oldYaml)) { copyFileSync(oldYaml, newYaml); unlinkSync(oldYaml) }
      if (existsSync(oldPcd)) { copyFileSync(oldPcd, newPcd); unlinkSync(oldPcd) }
      // 更新新 YAML 中的 image 字段
      if (existsSync(newYaml)) {
        writeFileSync(newYaml, `image: ${newName}.pgm\n` + readFileSync(newYaml, 'utf-8').replace(/^image:.*\n/m, ''))
      }
      // 尝试删除旧文件夹（如果为空）
      try { rmdirSync(oldFolder) } catch { /* 非空则保留 */ }
      console.log(`[gateway] Renamed map folder: ${oldFolder} -> ${newFolder}`)
    } else {
      // 旧文件夹不存在（可能是旧版数据库记录），直接创建新文件夹
      console.log(`[gateway] Old folder not found: ${oldFolder}, creating new: ${newFolder}`)
      mkdirSync(newFolder, { recursive: true })
    }
  } catch (e) { console.error('[gateway] rename file error:', e) }

  db.prepare('UPDATE maps SET name = ?, yaml_path = ?, pcd_path = ? WHERE id = ?').run(newName, newYaml, newPcd || row.pcd_path, req.params.id)
  broadcast('maps', db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all())
  res.json({ ok: true })
})

app.post('/api/maps/:id/switch', requireAuth, async (req, res) => {
  const row = db.prepare('SELECT * FROM maps WHERE id = ?').get(req.params.id) as { id: string; name: string; yaml_path: string; pcd_path: string } | undefined
  if (!row) {
    res.status(404).json({ error: 'Map not found' })
    return
  }
  // 调用切换 hook，同时传递 yaml_path 和 pcd_path
  const switchHook = process.env.MAP_SWITCH_HOOK
  if (switchHook) {
    execFile(switchHook, [row.yaml_path, row.pcd_path || '', row.name], (err) => {
      if (err) console.error('[gateway] MAP_SWITCH_HOOK failed:', err.message)
      else console.log('[gateway] MAP_SWITCH_HOOK done')
    })
  }
  db.prepare('UPDATE maps SET active = 0').run()
  db.prepare('UPDATE maps SET active = 1 WHERE id = ?').run(row.id)
  runtimeState.currentMapId = row.id
  broadcast('maps', db.prepare('SELECT * FROM maps ORDER BY created_at DESC').all())

  // MQ 通知: 地图切换成功（只发切到的那张）
  mqPublish('map_list', 'map_list', {
    switched_to: { id: row.id, name: row.name },
  }).catch(() => {})

  res.json({ ok: true })

  // 切换后从 PGM 重新生成 PNG+PGM，更新 telemetry（等 hook 完成文件拷贝）
  const MAP_ROOT = process.env.MAP_DIR || '/home/robot/go2_nav/lite_cog/system/map'
  setTimeout(() => {
    const currentPgm = path.join(MAP_ROOT, 'current.pgm')
    if (existsSync(currentPgm)) {
      regenerateMapPng(currentPgm)
    }
  }, 2000)
})

function broadcastNavPointsFromFiles() {
  broadcast('nav_points', readNavPointsFromFiles())
}

function writeNavPointFile(id: string, name: string, px: number, py: number, yaw: number, order: number) {
  const halfYawSin = Math.sin(yaw / 2)
  const halfYawCos = Math.cos(yaw / 2)
  const rec = {
    name,
    order,
    robot_pose: { pos_x: px, pos_y: py, pos_z: 0, ori_x: 0, ori_y: 0, ori_z: halfYawSin, ori_w: halfYawCos },
    option: { even_low_speed: false, even_medium_speed: false, uneven_high_step: false },
  }
  mkdirSync(TASK_DATA_DIR, { recursive: true })
  writeFileSync(path.join(TASK_DATA_DIR, `${id}.json`), JSON.stringify(rec, null, 4))
}

// Nav points — 直接读写 TASK_DATA JSON 文件
app.get('/api/nav-points', requireAuth, (_req, res) => {
  res.json({ navPoints: readNavPointsFromFiles() })
})

app.post('/api/nav-points', requireAuth, (req, res) => {
  const { name, x, y, yaw } = req.body ?? {}
  if (typeof x !== 'number' || typeof y !== 'number' || typeof yaw !== 'number') {
    res.status(400).json({ error: 'Invalid nav point payload' })
    return
  }
  const id = name || `P-${new Date().toISOString().slice(11, 19)}`
  const files = readdirSync(TASK_DATA_DIR).filter(f => f.endsWith('.json'))
  writeNavPointFile(id, name || id, x, y, yaw, files.length)
  broadcastNavPointsFromFiles()
  res.status(201).json({ id })
})

app.post(
  '/api/nav/goal',
  rateLimit({
    windowMs: navGoalRateWindowMs,
    limit: navGoalRateLimitCount,
    standardHeaders: 'draft-7',
    legacyHeaders: false,
    keyGenerator: (req) => (req as AuthedRequest).user?.sub ?? ipKeyGenerator(req.ip ?? ''),
    handler: (_req, res) => {
      res.status(429).json({ error: 'Too many nav goals, please retry shortly' })
    },
  }),
  requireAuth,
  async (req, res) => {
    const { x, y, yaw, frame_id } = req.body ?? {}
    if (typeof x !== 'number' || typeof y !== 'number' || typeof yaw !== 'number') {
      res.status(400).json({ error: 'Invalid nav goal payload' })
      return
    }
    const frameId = typeof frame_id === 'string' && frame_id.trim() ? frame_id.trim() : process.env.ROS_NAV_GOAL_FRAME_ID ?? 'map'
    await rosAdapter.publishNavGoal({ x, y, yaw, frameId })
    res.json({ ok: true })
  },
)

app.put('/api/nav-points/:id', requireAuth, (req, res) => {
  const { name, x, y, yaw } = req.body ?? {}
  const fp = path.join(TASK_DATA_DIR, `${req.params.id}.json`)
  if (!existsSync(fp)) { res.status(404).json({ error: 'Not found' }); return }
  try {
    const raw = JSON.parse(readFileSync(fp, 'utf-8'))
    if (typeof x === 'number') raw.robot_pose.pos_x = x
    if (typeof y === 'number') raw.robot_pose.pos_y = y
    if (typeof yaw === 'number') {
      raw.robot_pose.ori_z = Math.sin(yaw / 2)
      raw.robot_pose.ori_w = Math.cos(yaw / 2)
    }
    if (name) raw.name = name
    writeFileSync(fp, JSON.stringify(raw, null, 4))
    // rename if name changed
    if (name && name !== req.params.id) {
      renameSync(fp, path.join(TASK_DATA_DIR, `${name}.json`))
    }
  } catch { /* skip */ }
  broadcastNavPointsFromFiles()
  res.json({ ok: true })
})

app.delete('/api/nav-points/:id', requireAuth, (req, res) => {
  const fp = path.join(TASK_DATA_DIR, `${req.params.id}.json`)
  if (existsSync(fp)) unlinkSync(fp)
  broadcastNavPointsFromFiles()
  res.json({ ok: true })
})

// Virtual obstacles
app.get('/api/virtual-obstacles', requireAuth, (_req, res) => {
  const rows = parseObstacleRows(db.prepare('SELECT * FROM virtual_obstacles ORDER BY created_at DESC').all())
  res.json({ virtualObstacles: rows })
})

app.post('/api/virtual-obstacles', requireAuth, async (req, res) => {
  const { name, shape, data, enabled, map_id } = req.body ?? {}
  if (!['rect', 'circle', 'polygon'].includes(String(shape))) {
    res.status(400).json({ error: 'Invalid shape' })
    return
  }
  const id = newId()
  db.prepare(
    'INSERT INTO virtual_obstacles (id, name, shape, data, enabled, map_id, created_at) VALUES (?, ?, ?, ?, ?, ?, ?)',
  ).run(
    id,
    name ? String(name) : `OBS-${new Date().toISOString().slice(11, 19)}`,
    shape,
    JSON.stringify(data),
    enabled === false ? 0 : 1,
    map_id ?? runtimeState.currentMapId,
    new Date().toISOString(),
  )
  const payload = parseObstacleRows(db.prepare('SELECT * FROM virtual_obstacles ORDER BY created_at DESC').all())
  await rosAdapter.publishVirtualObstacles(payload)
  broadcast('virtual_obstacles', payload)
  res.status(201).json({ id })
})

app.put('/api/virtual-obstacles/:id', requireAuth, async (req, res) => {
  const { name, data, enabled } = req.body ?? {}
  db.prepare('UPDATE virtual_obstacles SET name = ?, data = ?, enabled = ? WHERE id = ?').run(name, JSON.stringify(data), enabled ? 1 : 0, req.params.id)
  const payload = parseObstacleRows(db.prepare('SELECT * FROM virtual_obstacles ORDER BY created_at DESC').all())
  await rosAdapter.publishVirtualObstacles(payload)
  broadcast('virtual_obstacles', payload)
  res.json({ ok: true })
})

app.delete('/api/virtual-obstacles/:id', requireAuth, async (req, res) => {
  db.prepare('DELETE FROM virtual_obstacles WHERE id = ?').run(req.params.id)
  const payload = parseObstacleRows(db.prepare('SELECT * FROM virtual_obstacles ORDER BY created_at DESC').all())
  await rosAdapter.publishVirtualObstacles(payload)
  broadcast('virtual_obstacles', payload)
  res.json({ ok: true })
})

// Lidar control
app.post('/api/lidar/start', requireAuth, async (_req, res) => {
  if (lidarProcess && !lidarProcess.killed) {
    res.json({ ok: true, message: '雷达已在运行' })
    return
  }
  try {
    lidarProcess = spawn('bash', [LIDAR_SCRIPT], { detached: true, stdio: 'ignore' })
    lidarProcess.on('exit', (code) => {
      console.log(`[gateway] LIDAR exited: ${code}`)
      lidarProcess = null
      runtimeState.lidarStatus = 'stopped'
      broadcast('lidar_status', 'stopped')
    })
    runtimeState.lidarStatus = 'running'
    broadcast('lidar_status', 'running')
    console.log('[gateway] LIDAR started')
    res.json({ ok: true })
  } catch (e) {
    res.status(500).json({ error: String(e) })
  }
})

app.post('/api/lidar/stop', requireAuth, async (_req, res) => {
  killProcess(lidarProcess, 'LIDAR')
  lidarProcess = null
  exec('pkill -f "[l]ivox_ros_driver" 2>/dev/null; pkill -f "[m]sg_MID360" 2>/dev/null', () => {})
  runtimeState.lidarStatus = 'stopped'
  broadcast('lidar_status', 'stopped')
  res.json({ ok: true })
})

// Motion control
app.post('/api/motion/start', requireAuth, async (_req, res) => {
  if (motionProcess && !motionProcess.killed) {
    res.json({ ok: true, message: '运动控制已在运行' })
    return
  }
  try {
    motionProcess = spawn('bash', [MOTION_SCRIPT], { detached: true, stdio: 'ignore' })
    motionProcess.on('exit', (code) => {
      console.log(`[gateway] MOTION exited: ${code}`)
      motionProcess = null
      runtimeState.motionStatus = 'stopped'
      broadcast('motion_status', 'stopped')
    })
    runtimeState.motionStatus = 'running'
    broadcast('motion_status', 'running')
    console.log('[gateway] MOTION started')
    res.json({ ok: true })
  } catch (e) {
    res.status(500).json({ error: String(e) })
  }
})

app.post('/api/motion/stop', requireAuth, async (_req, res) => {
  killProcess(motionProcess, 'MOTION')
  motionProcess = null
  exec('pkill -f "[g]o2_cmd_vel" 2>/dev/null; pkill -f "[u]nitree_sdk2" 2>/dev/null', () => {})
  runtimeState.motionStatus = 'stopped'
  broadcast('motion_status', 'stopped')
  res.json({ ok: true })
})

// Topology save: 保存拓扑点位到 Task.py 的 data 目录
const TASK_DATA_DIR = process.env.TASK_DATA_DIR || '/home/robot/go2_nav/lite_cog/pipeline/src/pipeline_tracking/data'

app.post('/api/topology/save', requireAuth, (req, res) => {
  const { points } = req.body ?? {}
  if (!Array.isArray(points)) {
    res.status(400).json({ error: 'points must be an array' })
    return
  }
  try {
    mkdirSync(TASK_DATA_DIR, { recursive: true })
    for (const f of readdirSync(TASK_DATA_DIR)) {
      if (f.endsWith('.json')) unlinkSync(path.join(TASK_DATA_DIR, f))
    }
    // 清除旧文件，重新写入
    for (const f of readdirSync(TASK_DATA_DIR)) {
      if (f.endsWith('.json')) unlinkSync(path.join(TASK_DATA_DIR, f))
    }
    for (let i = 0; i < points.length; i++) {
      const p = points[i]
      const half = (p.theta || 0) / 2
      const record = {
        name: p.name || `${i}`,
        order: i,
        robot_pose: {
          pos_x: p.x, pos_y: p.y, pos_z: 0,
          ori_x: 0, ori_y: 0,
          ori_z: Math.sin(half),
          ori_w: Math.cos(half),
        },
        option: { even_low_speed: false, even_medium_speed: false, uneven_high_step: false },
      }
      writeFileSync(path.join(TASK_DATA_DIR, `${i}.json`), JSON.stringify(record, null, 4))
      console.log(`[gateway] saved task point ${i}: x=${p.x}, y=${p.y}`)
    }
    console.log(`[gateway] topology saved: ${points.length} points -> ${TASK_DATA_DIR}`)
    broadcastNavPointsFromFiles()
    broadcast('topology_saved', { count: points.length })
    // 多点导航模式下重启 Task.py 读取新点位
    if (runtimeState.navMode === 'multi' && runtimeState.navStatus === 'running') {
      taskRestarting = true
      killProcess(taskProcess, 'TASK')
      taskProcess = null
      setTimeout(() => {
        startTaskProcess()
        taskRestarting = false
        console.log('[gateway] TASK restarted after topology save')
      }, 1000)
    }
    res.json({ ok: true, count: points.length })
  } catch (e) {
    res.status(500).json({ error: String(e) })
  }
})

// 启动 Task.py 进程（多点导航顺序执行）
function startTaskProcess() {
  const gen = navGeneration  // 记录当前 generation
  taskProcess = spawn('bash', ['-c', `source /opt/ros/noetic/setup.bash && source /home/robot/go2_nav/lite_cog/pipeline/devel/setup.bash && python3 ${TASK_SCRIPT} 2>&1 | tee /tmp/task.log`], {
    detached: true,
    stdio: 'ignore',
  })
  taskProcess.on('exit', (code) => {
    if (gen !== navGeneration) return  // 旧进程回调，忽略
    console.log(`[gateway] TASK exited with code ${code}`)
    taskProcess = null
    if (!taskRestarting) {
      runtimeState.navStatus = 'stopped'
      broadcast('nav_status', 'stopped')
    }
  })
  console.log(`[gateway] TASK started: ${TASK_SCRIPT}`)
}

// Nav command: start/stop/pause/resume navigation script
app.post('/api/nav/command', requireAuth, async (req, res) => {
  const { command } = req.body ?? {}

  if (command === 'stop') {
    killProcess(navProcess, 'NAV')
    navProcess = null
    killProcess(taskProcess, 'TASK')
    taskProcess = null
    exec('pkill -f "[v]elodyne_nodelet_manager" 2>/dev/null; pkill -f "[m]ove_base" 2>/dev/null; pkill -f "[T]ask.py" 2>/dev/null', () => {})
    runtimeState.navStatus = 'stopped'
    runtimeState.navMode = 'none'
    runtimeState.hdlRunning = false  // 立即反映，不等 poll
    broadcast('nav_status', 'stopped')
    res.json({ ok: true })
    return
  }

  if (command === 'pause') {
    taskRestarting = true  // 防止 exit 回调把状态改成 stopped
    killProcess(taskProcess, 'TASK')
    taskProcess = null
    runtimeState.navStatus = 'paused'
    broadcast('nav_status', 'paused')
    console.log('[gateway] NAV paused (Task.py stopped, localization+move_base kept)')
    res.json({ ok: true })
    return
  }

  if (command === 'resume') {
    if (runtimeState.navStatus !== 'paused') {
      res.status(400).json({ error: '当前未处于暂停状态' })
      return
    }
    taskRestarting = true
    taskProcess = null
    setTimeout(() => {
      startTaskProcess()
      taskRestarting = false
    }, 1000)
    runtimeState.navStatus = 'running'
    broadcast('nav_status', 'running')
    console.log('[gateway] NAV resumed (restarting Task.py)')
    res.json({ ok: true })
    return
  }

  const startOnly = command === 'nav-only'

  // 多点导航需要先保存拓扑点
  if (!startOnly) {
    const existing = readdirSync(TASK_DATA_DIR).filter(f => f.endsWith('.json'))
    if (existing.length === 0) {
      res.status(400).json({ error: '请先在"地图编辑"中添加并"发布点位"，再启动多点导航' })
      return
    }
  }

  runtimeState.navMode = startOnly ? 'single' : 'multi'
  // 递增 generation，旧进程的 exit 回调将被忽略
  const gen = ++navGeneration
  killProcess(navProcess, 'NAV')
  navProcess = null
  killProcess(taskProcess, 'TASK')
  taskProcess = null
  // 清理残留导航进程（保留 hdl_localization，避免丢失定位）
  exec('pkill -f "[m]ove_base" 2>/dev/null; pkill -f "[T]ask.py" 2>/dev/null', () => {})
  // 等待进程退出
  await new Promise(r => setTimeout(r, 1500))

  try {
    navProcess = spawn('bash', [NAV_SCRIPT], {
      detached: true,
      stdio: 'ignore',
    })
    navProcess.on('exit', (code) => {
      if (gen !== navGeneration) return  // 旧进程回调，忽略
      console.log(`[gateway] NAV exited with code ${code}`)
      navProcess = null
      if (!startOnly) {
        runtimeState.navStatus = 'stopped'
        broadcast('nav_status', 'stopped')
      }
    })
    console.log(`[gateway] NAV started (${command}): ${NAV_SCRIPT}`)

    if (!startOnly) {
      // 多点导航：延迟 8 秒等 hdl_localization + move_base 完全启动，然后启动 Task.py
      setTimeout(() => {
        startTaskProcess()
      }, 8000)
    }

    runtimeState.navStatus = 'running'
    broadcast('nav_status', 'running')
    res.json({ ok: true })
  } catch (e) {
    runtimeState.navStatus = 'stopped'
    res.status(500).json({ error: String(e) })
  }
})

// 调整多点导航顺序
app.post('/api/nav/reorder', requireAuth, (req, res) => {
  const { order } = req.body ?? {}
  if (!Array.isArray(order) || order.length === 0) {
    res.status(400).json({ error: 'order must be a non-empty array of point names' })
    return
  }
  try {
    // 读取现有 JSON 文件，建立 名称→内容 映射（在删除前全部读取）
    const existingFiles = readdirSync(TASK_DATA_DIR).filter(f => f.endsWith('.json'))
    // 同时建立 filename→content 和 display_name→content 映射
    const pointMap = new Map<string, string>() // name -> JSON content
    for (const f of existingFiles) {
      const fp = path.join(TASK_DATA_DIR, f)
      const content = readFileSync(fp, 'utf-8')
      pointMap.set(f.replace('.json', ''), content)
      // 如果 JSON 里有 name 字段，也以其为 key 建立映射
      try {
        const rec = JSON.parse(content)
        if (typeof rec.name === 'string') {
          pointMap.set(rec.name, content)
        }
      } catch { /* skip */ }
    }

    // 清除所有旧 JSON 文件
    for (const f of existingFiles) {
      unlinkSync(path.join(TASK_DATA_DIR, f))
    }

    // 按新顺序重新写入
    for (let i = 0; i < order.length; i++) {
      const name = order[i]
      const content = pointMap.get(name)
      if (content) {
        const record = JSON.parse(content)
        record.order = i
        // 确保 name 字段存在（向前兼容）
        if (!record.name) record.name = name
        writeFileSync(path.join(TASK_DATA_DIR, `${i}.json`), JSON.stringify(record, null, 4))
      }
    }

    console.log(`[gateway] nav order updated: ${order.join(' -> ')}`)
    broadcast('topology_saved', { count: order.length })

    // 如果导航正在运行，重启 Task.py
    if (runtimeState.navStatus === 'running' && runtimeState.navMode === 'multi') {
      taskRestarting = true
      killProcess(taskProcess, 'TASK')
      taskProcess = null
      setTimeout(() => {
        startTaskProcess()
        taskRestarting = false
      }, 1000)
    }

    res.json({ ok: true, count: order.length })
  } catch (e) {
    res.status(500).json({ error: String(e) })
  }
})

// Publish occupancy grid to /map
app.post('/api/map/publish', requireAuth, async (req, res) => {
  const { width, height, resolution, origin, data } = req.body ?? {}
  if (!width || !height || !resolution || !origin || !Array.isArray(data)) {
    res.status(400).json({ error: 'Invalid occupancy grid payload' })
    return
  }
  const w = Number(width), h = Number(height)
  const resVal = Number(resolution)
  const ox = Number(origin?.x ?? 0), oy = Number(origin?.y ?? 0)
  const numData: number[] = data.map((v: unknown) => Number(v))

  // Write edited grid to PGM+PNG on disk (so frontend sees changes immediately)
  const staticMapsDir = path.resolve(process.cwd(), 'data', 'maps')
  mkdirSync(staticMapsDir, { recursive: true })

  // Publish to /map via rosbridge (only small maps; large maps use map_server reload instead)
  const cells = w * h
  if (cells <= LARGE_MAP_CELL_THRESHOLD) {
    await rosAdapter.publishOccupancyGrid({ width: w, height: h, resolution: resVal, origin: { x: ox, y: oy }, data: numData })
  }

  // Write PGM + PNG from edited data, update telemetry
  try {
    // Build PGM
    const pgmHeader = Buffer.from(`P5\n${w} ${h}\n255\n`)
    const pgmPixels = Buffer.alloc(w * h)
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const srcIdx = (h - 1 - y) * w + x
        const v = numData[srcIdx] ?? -1
        pgmPixels[y * w + x] = v === 100 ? 0 : v === 0 ? 254 : 205
      }
    }
    writeFileSync(path.join(staticMapsDir, 'live_map.pgm'), Buffer.concat([pgmHeader, pgmPixels]))

    // Build PNG
    const rawRows = Buffer.alloc((w + 1) * h)
    for (let y = 0; y < h; y++) {
      rawRows[y * (w + 1)] = 0
      for (let x = 0; x < w; x++) {
        const srcIdx = (h - 1 - y) * w + x
        const v = numData[srcIdx] ?? -1
        rawRows[y * (w + 1) + 1 + x] = v === 100 ? 0 : v === 0 ? 254 : 205
      }
    }
    const sig = Buffer.from([137, 80, 78, 71, 13, 10, 26, 10])
    const ihdrData = Buffer.alloc(13)
    ihdrData.writeUInt32BE(w, 0); ihdrData.writeUInt32BE(h, 4)
    ihdrData[8] = 8; ihdrData[9] = 0; ihdrData[10] = 0; ihdrData[11] = 0; ihdrData[12] = 0
    const png = Buffer.concat([sig, pngChunk('IHDR', ihdrData), pngChunk('IDAT', deflateSync(rawRows)), pngChunk('IEND', Buffer.alloc(0))])
    writeFileSync(path.join(staticMapsDir, 'live_map.png'), png)

    // Write to active/ dir + reload map_server so static_layer picks up edits
    const MAP_ROOT = process.env.MAP_DIR || '/home/robot/go2_nav/lite_cog/system/map'
    const activeDir = path.join(MAP_ROOT, 'active')
    mkdirSync(activeDir, { recursive: true })
    copyFileSync(path.join(staticMapsDir, 'live_map.pgm'), path.join(activeDir, 'current.pgm'))
    writeFileSync(path.join(activeDir, 'current.yaml'), `image: current.pgm\nresolution: ${resVal}\norigin: [${ox}, ${oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n`)
    execFile('bash', ['-c', 'source /opt/ros/noetic/setup.bash && rosservice call /change_map ' + path.join(activeDir, 'current.yaml') + ' 2>/dev/null'], (err) => {
      if (err) console.error('[map-publish] map_server reload failed:', err.message)
      else console.log('[map-publish] map_server reloaded → /map updated → static_layer sees edits')
    })

    // Update runtimeState so telemetry broadcasts the change
    const isLarge = cells > LARGE_MAP_CELL_THRESHOLD
    runtimeState.map = {
      width: w, height: h, resolution: resVal,
      origin: { x: ox, y: oy },
      mapUrl: `/static/maps/live_map.png?t=${Date.now()}`,
      data: isLarge ? [] : numData,
    }
    runtimeState.mapUpdatedAt = new Date().toISOString()
    console.log(`[map-publish] Saved edited map (${w}x${h}), PNG ${(png.length/1024).toFixed(0)}KB`)
  } catch (e) {
    console.error('[map-publish] Failed to save edited map:', e)
  }

  res.json({ ok: true })
})

// 2D Nav Goal (like rviz)
app.post('/api/nav/simple-goal', requireAuth, async (req, res) => {
  const { x, y, yaw, frame_id } = req.body ?? {}
  if (typeof x !== 'number' || typeof y !== 'number' || typeof yaw !== 'number') {
    res.status(400).json({ error: 'x, y, yaw required' })
    return
  }
  await rosAdapter.publishSimpleGoal({
    x: Number(x),
    y: Number(y),
    yaw: Number(yaw),
    frameId: String(frame_id ?? 'map'),
  })
  runtimeState.navGoal = { x: Number(x), y: Number(y), yaw: Number(yaw) }
  broadcast('nav_goal', runtimeState.navGoal)
  res.json({ ok: true })
})

// Initial pose (2D Pose Estimate)
app.post('/api/initialpose', requireAuth, async (req, res) => {
  const { x, y, yaw, frame_id } = req.body ?? {}
  if (typeof x !== 'number' || typeof y !== 'number' || typeof yaw !== 'number') {
    res.status(400).json({ error: 'x, y, yaw required' })
    return
  }
  await rosAdapter.publishInitialPose({
    x: Number(x),
    y: Number(y),
    yaw: Number(yaw),
    frameId: String(frame_id ?? 'map'),
  })
  res.json({ ok: true })
})

// Nav goal tolerance (runtime dynamic reconfigure)
app.post('/api/nav/tolerance', requireAuth, async (req, res) => {
  const { xy_tolerance } = req.body ?? {}
  if (typeof xy_tolerance !== 'number' || xy_tolerance < 0.01 || xy_tolerance > 5) {
    res.status(400).json({ error: 'xy_tolerance must be 0.01–5 (meters)' })
    return
  }
  try {
    execSync(
      `bash -c 'source /opt/ros/noetic/setup.bash && rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS xy_goal_tolerance ${Number(xy_tolerance)}'`,
      { timeout: 10000 }
    )
    console.log(`[gateway] xy_goal_tolerance set to ${xy_tolerance}`)
    res.json({ ok: true, xy_tolerance: Number(xy_tolerance) })
  } catch (e: any) {
    console.error('[gateway] Failed to set xy_goal_tolerance:', e.message)
    res.status(500).json({ error: 'Failed to set tolerance: ' + e.message })
  }
})

// Obstacle distance (runtime dynamic reconfigure)
app.post('/api/nav/obstacle-dist', requireAuth, async (req, res) => {
  const { min_dist } = req.body ?? {}
  if (typeof min_dist !== 'number' || min_dist < 0.05 || min_dist > 2) {
    res.status(400).json({ error: 'min_dist must be 0.05–2 (meters)' })
    return
  }
  try {
    execSync(
      `bash -c 'source /opt/ros/noetic/setup.bash && rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist ${Number(min_dist)}'`,
      { timeout: 10000 }
    )
    console.log(`[gateway] min_obstacle_dist set to ${min_dist}`)
    res.json({ ok: true, min_obstacle_dist: Number(min_dist) })
  } catch (e: any) {
    console.error('[gateway] Failed to set min_obstacle_dist:', e.message)
    res.status(500).json({ error: 'Failed to set obstacle distance: ' + e.message })
  }
})

// Unified nav param — dynamic_reconfigure (immediate effect, TEB params)
app.post('/api/nav/param/reconfigure', requireAuth, async (req, res) => {
  const { key, value } = req.body ?? {}
  if (!key || typeof value !== 'number') {
    res.status(400).json({ error: 'key and value required' })
    return
  }

  // ── YAML persistence first (fast, reliable, decoupled from ROS) ──
  const TEB_YAML = '/home/robot/go2_nav/lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml'
  try {
    // [-0-9.]* handles negative values like -0.1 (old regex [0-9.]* skipped leading -)
    execSync(`sed -i "s/^\\(\\\\s*${key}:\\\\s*\\)[-0-9.]*/\\\\1${Number(value)}/" ${TEB_YAML}`, { timeout: 3000 })
    console.log(`[gateway] yaml updated: ${key}=${value} in ${TEB_YAML}`)
  } catch { /* YAML update is best-effort */ }

  try {
    const cmd = `bash -c 'source /opt/ros/noetic/setup.bash && rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS ${String(key)} ${Number(value)}'`
    execSync(cmd, { timeout: 10000 })
    console.log(`[gateway] reconfigure ${key}=${value}`)
    res.json({ ok: true, key, value })
  } catch (e: any) {
    console.error(`[gateway] reconfigure failed ${key}:`, e.message)
    res.status(500).json({ error: e.message })
  }
})

// Unified nav param — rosparam (needs costmap clear or nav restart)
// NOTE: inflation_radius / cost_scaling_factor use dynamic_reconfigure for immediate effect.
//       obstacle_range / min_z / max_z trigger a move_base restart (3-5s interruption).
const RESTART_MB_SCRIPT = process.env.RESTART_MB_SCRIPT || '/home/robot/go2_nav/lite_cog/system/scripts/nav/restart_move_base.sh'
let restartMbTimer: NodeJS.Timeout | null = null
const RESTART_MB_DEBOUNCE_MS = 2000  // batch rapid slider changes into one restart

function scheduleRestartMoveBase() {
  if (restartMbTimer) clearTimeout(restartMbTimer)
  restartMbTimer = setTimeout(() => {
    restartMbTimer = null
    console.log('[gateway] restarting move_base for STVL param changes ...')
    execFile('bash', [RESTART_MB_SCRIPT], (err, stdout, stderr) => {
      if (err) console.error('[gateway] move_base restart failed:', stderr || err.message)
      else console.log('[gateway] move_base restart OK:', stdout.trim())
    })
  }, RESTART_MB_DEBOUNCE_MS)
}

app.post('/api/nav/param/rosparam', requireAuth, async (req, res) => {
  const { key, value } = req.body ?? {}
  if (!key || typeof value !== 'number') {
    res.status(400).json({ error: 'key and value required' })
    return
  }

  // All costmap params now use rosparam + YAML + debounced restart
  const DYN_RECONFIG_LAYERS: Record<string, string> = {}

  const dynLayer = DYN_RECONFIG_LAYERS[key]
  const needsRestart = !dynLayer  // STVL params need move_base restart

  // Map frontend key to rosparam paths (global + local costmap)
  const keyToParams: Record<string, string[]> = {
    obstacle_range:       ['/move_base/global_costmap/livox_lidar/obstacle_range', '/move_base/local_costmap/livox_lidar/obstacle_range'],
    min_obstacle_height:  ['/move_base/global_costmap/livox_lidar/min_z', '/move_base/local_costmap/livox_lidar/min_z'],
    max_obstacle_height:  ['/move_base/global_costmap/livox_lidar/max_z', '/move_base/local_costmap/livox_lidar/max_z'],
    inflation_radius:     ['/move_base/global_costmap/sob_layer/inflation_radius', '/move_base/local_costmap/sob_layer/inflation_radius'],
    cost_scaling_factor:  ['/move_base/global_costmap/sob_layer/cost_scaling_factor', '/move_base/local_costmap/sob_layer/cost_scaling_factor'],
  }

  try {
    if (dynLayer) {
      // ── dynamic_reconfigure for inflation-layer params (immediate) ──
      const servers = [
        `/move_base/local_costmap/${dynLayer}`,
        `/move_base/global_costmap/${dynLayer}`,
      ]
      // Run local + global costmap dynparam in parallel for speed
      const dynCmd = `bash -c 'source /opt/ros/noetic/setup.bash && rosrun dynamic_reconfigure dynparam set ${servers[0]} ${key} ${Number(value)} & rosrun dynamic_reconfigure dynparam set ${servers[1]} ${key} ${Number(value)} & wait'`
      execSync(dynCmd, { timeout: 10000 })
      console.log(`[gateway] dynparam ${servers[0]} + ${servers[1]} ${key}=${value}`)
    } else {
      // ── rosparam set for STVL observation-source params ──
      const paths = keyToParams[key] || [key]
      if (paths.length >= 2) {
        execSync(`bash -c 'source /opt/ros/noetic/setup.bash && rosparam set ${paths[0]} ${Number(value)} & rosparam set ${paths[1]} ${Number(value)} & wait'`, { timeout: 5000 })
      } else {
        execSync(`bash -c 'source /opt/ros/noetic/setup.bash && rosparam set ${paths[0]} ${Number(value)}'`, { timeout: 5000 })
      }
      console.log(`[gateway] rosparam ${paths.join(', ')} = ${value}`)
    }

    // ── Also persist to YAML config file so values survive nav restart ──
    const TEB_KEYS = new Set(['xy_goal_tolerance', 'yaw_goal_tolerance'])
    const COSTMAP_YAML = process.env.COSTMAP_YAML || '/home/robot/go2_nav/lite_cog/nav/src/navigation/config/common_costmap_params.yaml'
    const TEB_YAML = '/home/robot/go2_nav/lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml'
    const YAML_FILE = TEB_KEYS.has(key) ? TEB_YAML : COSTMAP_YAML
    // Use frontend key directly as YAML key (frontend keys match YAML keys).
    // Do NOT derive from rosparam path suffix — e.g. min_obstacle_height maps to
    // rosparam .../min_z but YAML uses min_obstacle_height (different from min_z).
    const yamlKey = key
    try {
      // [-0-9.]* handles negative values like -0.1 (old regex [0-9.]* skipped leading -)
      execSync(`sed -i "s/^\\(\\\\s*${yamlKey}:\\\\s*\\)[-0-9.]*/\\\\1${Number(value)}/" ${YAML_FILE}`, { timeout: 3000 })
      console.log(`[gateway] yaml updated: ${yamlKey}=${value} in ${YAML_FILE}`)
    } catch { /* YAML update is best-effort */ }

    // ── Restart move_base for STVL params (debounced) ──
    if (needsRestart) {
      scheduleRestartMoveBase()
    }

    res.json({ ok: true, key, value, restarted: needsRestart ? 'scheduled' : false })
  } catch (e: any) {
    console.error(`[gateway] param failed ${key}:`, e.message)
    res.status(500).json({ error: e.message })
  }
})

// Clear costmaps (makes rosparam changes take effect)
app.post('/api/nav/costmap-clear', requireAuth, async (_req, res) => {
  try {
    execSync(
      `bash -c 'source /opt/ros/noetic/setup.bash && rosservice call /move_base/clear_costmaps "{}"'`,
      { timeout: 10000 }
    )
    console.log('[gateway] costmaps cleared')
    res.json({ ok: true })
  } catch (e: any) {
    console.error('[gateway] Failed to clear costmaps:', e.message)
    res.status(500).json({ error: e.message })
  }
})

// Cmd vel
app.post('/api/cmd-vel', requireAuth, async (req, res) => {
  const linear = Number(req.body?.linear ?? 0)
  const angular = Number(req.body?.angular ?? 0)
  await rosAdapter.publishCmdVel({ linear, angular })
  res.json({ ok: true })
})

// WebSocket
wss.on('connection', (ws) => {
  ws.send(JSON.stringify({ event: 'snapshot', payload: snapshot() }))
})

// ── hdl_localization 节点检测 ──
let lastHdlCheckAt = 0
const HDL_CHECK_INTERVAL_MS = 2000

function checkHdlRunning() {
  const now = Date.now()
  if (now - lastHdlCheckAt < HDL_CHECK_INTERVAL_MS) return
  lastHdlCheckAt = now
  exec('pgrep -f "^/opt/ros/noetic/lib/nodelet/nodelet.*velodyne_nodelet_manager" >/dev/null 2>&1 && echo running || echo stopped',
    { timeout: 3000 },
    (_err, stdout) => {
      const running = (stdout || '').trim() === 'running'
      if (running !== runtimeState.hdlRunning) {
        runtimeState.hdlRunning = running
        console.log(`[gateway] hdl_localization ${running ? '🟢 running' : '🔴 stopped'}`)
      }
    })
}

// Telemetry broadcast — differential: only send fields that changed
let lastTelemetry: Record<string, unknown> = {}
setInterval(() => {
  checkHdlRunning()
  if (rosAdapter.getStatus().mode === 'mock') {
    // MQ 有实时位姿时不再用 mock 数据覆盖
    if (Date.now() - lastMqPoseAt > 3000) {
      updateMockPoseAndLidar()
    }
  }

  // 差分构建：只包含变化的字段
  const diff: Record<string, unknown> = {}
  const fields: Array<{ key: string; val: unknown }> = [
    { key: 'tfPose',       val: runtimeState.tfPose },
    // lidar 在前端已注释不渲染，不发送省带宽（26KB/帧）
    { key: 'globalPlan',   val: runtimeState.globalPlan },
    { key: 'localPlan',    val: runtimeState.localPlan },
    { key: 'voxelGrid',    val: runtimeState.voxelGrid },
    { key: 'navStatus',    val: runtimeState.navStatus },
    { key: 'navMode',      val: runtimeState.navMode },
    { key: 'controlSource', val: runtimeState.controlSource },
    { key: 'battery',      val: runtimeState.battery },
    { key: 'bodyState',    val: runtimeState.bodyState },
    { key: 'hdlRunning',   val: runtimeState.hdlRunning },
    { key: 'lastTfAt',     val: runtimeState.lastTfAt },
    { key: 'lastVoxelAt',  val: runtimeState.lastVoxelAt },
    { key: 'mapUpdatedAt', val: runtimeState.mapUpdatedAt },
  ]

  // 地图数据大，只在 mapUpdatedAt 变化时发送
  // 大尺寸地图（≥200万栅格）只发送元数据+PGM URL，前端通过 HTTP 加载图片
  if (runtimeState.mapUpdatedAt !== lastTelemetry['mapUpdatedAt'] && (runtimeState.map.data.length > 0 || runtimeState.map.mapUrl)) {
    const cells = runtimeState.map.width * runtimeState.map.height
    if (cells > LARGE_MAP_CELL_THRESHOLD) {
      diff['map'] = {
        width: runtimeState.map.width,
        height: runtimeState.map.height,
        resolution: runtimeState.map.resolution,
        origin: runtimeState.map.origin,
        mapUrl: runtimeState.map.mapUrl,
        data: [],
      }
    } else {
      diff['map'] = runtimeState.map
    }
  }

  for (const { key, val } of fields) {
    // 数组/对象用 JSON 比较；标量直接比较
    const prev = lastTelemetry[key]
    const changed = Array.isArray(val) || (typeof val === 'object' && val !== null)
      ? JSON.stringify(val) !== JSON.stringify(prev)
      : val !== prev
    if (changed) {
      diff[key] = val
      lastTelemetry[key] = Array.isArray(val) ? [...(val as unknown[])] : (typeof val === 'object' && val !== null ? { ...(val as Record<string, unknown>) } : val)
    }
  }

  diff.ts = runtimeState.lastSnapshotAt
  broadcast('telemetry', diff)
}, 200)  // 5Hz — 外部设备更新更流畅

// Bootstrap
async function bootstrap() {
  mkdirSync(path.resolve(process.cwd(), 'config'), { recursive: true })
  // Generate live map BEFORE starting server — avoids frontend seeing stale data
  const MAP_ROOT = process.env.MAP_DIR || '/home/robot/go2_nav/lite_cog/system/map'
  const currentPgm = path.join(MAP_ROOT, 'current.pgm')
  if (existsSync(currentPgm)) {
    regenerateMapPng(currentPgm)
  }
  await rosAdapter.start()
  mqConnect()
  server.listen(PORT, () => {
    console.log(`Gateway listening on :${PORT}`)
  })
}

void bootstrap()
