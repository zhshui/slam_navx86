import { useEffect, useRef, useState, useCallback } from 'react'
import { toast } from 'react-toastify'
import { apiBase, apiRequest, AuthError } from '../api/gatewayApi'
import type { Snapshot, NavPoint, VirtualObstacle, MapRecord } from '../types/GatewayTypes'

export interface UseGatewayReturn {
  token: string | null
  snapshot: Snapshot | null
  wsState: 'connecting' | 'connected' | 'disconnected'
  refreshSnapshot: () => Promise<void>
  runAction: (path: string) => Promise<void>
  createNavPoint: (p: { x: number; y: number; yaw: number }) => Promise<void>
  publishNavGoal: (p: { x: number; y: number; yaw: number }) => Promise<void>
  createObstacle: (shape: string, data: unknown) => Promise<void>
  deleteItem: (path: string) => Promise<void>
  toggleObstacle: (id: string, enable: boolean) => Promise<void>
  setToken: (t: string | null) => void
}

export function useGateway(authToken: string | null, onAuthError?: () => void): UseGatewayReturn {
  const [token, setToken] = useState<string | null>(authToken)
  const [snapshot, setSnapshot] = useState<Snapshot | null>(null)
  const [wsState, setWsState] = useState<'connecting' | 'connected' | 'disconnected'>('disconnected')
  const wsRef = useRef<WebSocket | null>(null)
  // 遥测节流：最多 5Hz 更新
  const lastTelemetryAt = useRef(0)
  const pendingTelemetry = useRef<Record<string, unknown> | null>(null)
  const telemetryTimer = useRef<ReturnType<typeof setTimeout> | null>(null)

  // Sync token from props
  useEffect(() => {
    setToken(authToken)
  }, [authToken])

  // WebSocket connection
  useEffect(() => {
    if (!token) {
      setSnapshot(null)
      setWsState('disconnected')
      return
    }
    let cancelled = false
    let retry = 500
    const wsUrl = apiBase
      ? `${apiBase.replace('http', 'ws')}/ws`
      : `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws`

    const connect = async () => {
      try {
        const s = await apiRequest<Snapshot>('/api/snapshot', token)
        if (!cancelled) setSnapshot(s)
      } catch (e) {
        if (e instanceof AuthError) {
          // Token 无效（过期、密钥变更等），触发登出
          console.warn('[useGateway] Auth error, clearing token:', e.message)
          onAuthError?.()
          return
        }
        // 其他错误（网络、服务未就绪等）忽略，等待重试
      }
      if (cancelled) return
      setWsState('connecting')
      const ws = new WebSocket(wsUrl)
      wsRef.current = ws
      ws.onopen = () => {
        retry = 500
        setWsState('connected')
      }
      ws.onmessage = (evt) => {
        const msg = JSON.parse(evt.data) as { event: string; payload: unknown }

        // 非遥测事件立即处理
        if (msg.event !== 'telemetry') {
          setSnapshot((prev) => {
            if (msg.event === 'snapshot') return msg.payload as Snapshot
            if (!prev) return prev
            if (msg.event === 'nav_points') return { ...prev, navPoints: msg.payload as NavPoint[] }
            if (msg.event === 'virtual_obstacles') return { ...prev, virtualObstacles: msg.payload as VirtualObstacle[] }
            if (msg.event === 'maps') return { ...prev, maps: msg.payload as MapRecord[] }
            if (msg.event === 'mapping_status') return { ...prev, runtime: { ...prev.runtime, mappingStatus: String(msg.payload) } }
            if (msg.event === 'mapping_complete') {
              const p = msg.payload as { name: string; pcdFile: string }
              toast.success(`✅ 建图完成！已保存「${p.name}」(PCD: ${p.pcdFile})`, { autoClose: 8000 })
              return prev
            }
            if (msg.event === 'nav_goal') return { ...prev, runtime: { ...prev.runtime, navGoal: msg.payload as { x: number; y: number; yaw: number } } }
            if (msg.event === 'nav_status') return { ...prev, runtime: { ...prev.runtime, navStatus: String(msg.payload) as 'idle' | 'running' | 'stopped' } }
            if (msg.event === 'lidar_status') return { ...prev, runtime: { ...prev.runtime, lidarStatus: String(msg.payload) as 'idle' | 'running' | 'stopped' } }
            if (msg.event === 'motion_status') return { ...prev, runtime: { ...prev.runtime, motionStatus: String(msg.payload) as 'idle' | 'running' | 'stopped' } }
            return prev
          })
          return
        }

        // 遥测事件：节流到 5Hz (200ms)
        pendingTelemetry.current = msg.payload as Record<string, unknown>
        const now = Date.now()
        const elapsed = now - lastTelemetryAt.current
        const flush = () => {
          if (!pendingTelemetry.current) return
          const t = pendingTelemetry.current as Partial<Snapshot['runtime'] & {
            tfPose: Snapshot['runtime']['tfPose']
            lidar: Snapshot['runtime']['lidar']
            map: Snapshot['runtime']['map']
            mapUpdatedAt: string | null
            globalPlan: Array<{ x: number; y: number }>
            localPlan: Array<{ x: number; y: number }>
            voxelGrid: Array<{ x: number; y: number }>
            lastVoxelAt: string | null
            navStatus?: string
          }>
          pendingTelemetry.current = null
          setSnapshot((prev) => {
            if (!prev) return prev
            return {
              ...prev,
              runtime: {
                ...prev.runtime,
                tfPose: t.tfPose ?? prev.runtime.tfPose,
                lastTfAt: t.lastTfAt ?? prev.runtime.lastTfAt,
                lastVoxelAt: t.lastVoxelAt ?? prev.runtime.lastVoxelAt,
                lidar: t.lidar ?? prev.runtime.lidar,
                map: t.map ?? prev.runtime.map,
                mapUpdatedAt: t.mapUpdatedAt ?? prev.runtime.mapUpdatedAt,
                globalPlan: t.globalPlan ?? prev.runtime.globalPlan,
                localPlan: t.localPlan ?? prev.runtime.localPlan,
                voxelGrid: t.voxelGrid ?? prev.runtime.voxelGrid,
                navStatus: (t.navStatus ?? prev.runtime.navStatus) as 'idle' | 'running' | 'paused' | 'stopped',
                navMode: (t.navMode ?? prev.runtime.navMode) as 'none' | 'single' | 'multi',
                lastSnapshotAt: new Date().toISOString(),
              },
            }
          })
        }
        if (elapsed >= 200) {
          lastTelemetryAt.current = now
          if (telemetryTimer.current) { clearTimeout(telemetryTimer.current); telemetryTimer.current = null }
          flush()
        } else if (!telemetryTimer.current) {
          telemetryTimer.current = setTimeout(() => {
            telemetryTimer.current = null
            lastTelemetryAt.current = Date.now()
            flush()
          }, 200 - elapsed)
        }
      }
      ws.onclose = () => {
        setWsState('disconnected')
        if (!cancelled) {
          setTimeout(connect, retry)
          retry = Math.min(5000, retry * 2)
        }
      }
    }

    void connect()

    return () => {
      cancelled = true
      wsRef.current?.close()
      if (telemetryTimer.current) { clearTimeout(telemetryTimer.current); telemetryTimer.current = null }
    }
  }, [token])

  const refreshSnapshot = useCallback(async () => {
    if (!token) return
    const s = await apiRequest<Snapshot>('/api/snapshot', token)
    setSnapshot(s)
  }, [token])

  const runAction = useCallback(async (path: string) => {
    if (!token) return
    await apiRequest(path, token, { method: 'POST' })
    await refreshSnapshot()
  }, [token, refreshSnapshot])

  const createNavPoint = useCallback(async (p: { x: number; y: number; yaw: number }) => {
    if (!token) return
    await apiRequest('/api/nav-points', token, {
      method: 'POST',
      body: JSON.stringify({ x: p.x, y: p.y, yaw: p.yaw, name: `P-${Date.now()}` }),
    })
    await refreshSnapshot()
  }, [token, refreshSnapshot])

  const publishNavGoal = useCallback(async (p: { x: number; y: number; yaw: number }) => {
    if (!token) return
    await apiRequest('/api/nav/goal', token, {
      method: 'POST',
      body: JSON.stringify({ x: p.x, y: p.y, yaw: p.yaw, frame_id: 'map' }),
    })
  }, [token])

  const createObstacle = useCallback(async (shape: string, data: unknown) => {
    if (!token) return
    await apiRequest('/api/virtual-obstacles', token, {
      method: 'POST',
      body: JSON.stringify({ name: `OBS-${Date.now()}`, shape, data, enabled: true }),
    })
    await refreshSnapshot()
  }, [token, refreshSnapshot])

  const deleteItem = useCallback(async (path: string) => {
    if (!token) return
    await apiRequest(path, token, { method: 'DELETE' })
    await refreshSnapshot()
  }, [token, refreshSnapshot])

  const toggleObstacle = useCallback(async (id: string, enable: boolean) => {
    if (!token || !snapshot) return
    const hit = snapshot.virtualObstacles.find((o) => o.id === id)
    if (!hit) return
    await apiRequest(`/api/virtual-obstacles/${id}`, token, {
      method: 'PUT',
      body: JSON.stringify({ name: hit.name, data: hit.data, enabled: enable }),
    })
    await refreshSnapshot()
  }, [token, snapshot, refreshSnapshot])

  return {
    token,
    snapshot,
    wsState,
    refreshSnapshot,
    runAction,
    createNavPoint,
    publishNavGoal,
    createObstacle,
    deleteItem,
    toggleObstacle,
    setToken,
  }
}
