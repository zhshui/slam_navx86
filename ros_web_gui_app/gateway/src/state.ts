import { randomUUID } from 'node:crypto'

/** Maps with cells above this threshold skip sending raw data via WebSocket;
 *  the frontend loads the PGM image via HTTP instead. */
export const LARGE_MAP_CELL_THRESHOLD = 2_000_000

export interface OccupancySnapshot {
  width: number
  height: number
  resolution: number
  origin: { x: number; y: number }
  data: number[]
  mapUrl?: string | null
}

export const runtimeState = {
  rosConnected: false,
  mappingStatus: 'idle' as 'idle' | 'running' | 'stopped',
  mappingPauseSupported: false,
  navStatus: 'idle' as 'idle' | 'running' | 'paused' | 'stopped',
  navMode: 'none' as 'none' | 'single' | 'multi',
  navGoal: null as { x: number; y: number; yaw: number } | null,
  lidarStatus: 'stopped' as 'idle' | 'running' | 'stopped',
  motionStatus: 'stopped' as 'idle' | 'running' | 'stopped',
  currentMapId: null as string | null,
  map: { width: 0, height: 0, resolution: 0.05, origin: { x: 0, y: 0 }, data: [] as number[], mapUrl: null as string | null },
  tfPose: { x: 0, y: 0, yaw: 0, frame: 'map' },
  lidar: [] as Array<{ x: number; y: number }>,
  globalPlan: [] as Array<{ x: number; y: number }>,
  localPlan: [] as Array<{ x: number; y: number }>,
  voxelGrid: [] as Array<{ x: number; y: number }>,
  mapUpdatedAt: null as string | null,
  lastMapAt: null as string | null,
  lastTfAt: null as string | null,
  lastLidarAt: null as string | null,
  lastGlobalPlanAt: null as string | null,
  lastLocalPlanAt: null as string | null,
  lastVoxelAt: null as string | null,
  hdlRunning: false,
  lastSnapshotAt: new Date().toISOString(),
}

export function updateMockPoseAndLidar() {
  const now = new Date().toISOString()
  const t = Date.now() / 1000
  runtimeState.tfPose = {
    x: Math.cos(t / 4),
    y: Math.sin(t / 4),
    yaw: (t / 4) % (Math.PI * 2),
    frame: 'map',
  }
  runtimeState.lidar = Array.from({ length: 120 }, (_, i) => {
    const a = (i / 120) * Math.PI * 2
    const r = 1.7 + 0.2 * Math.sin(t * 2 + i)
    return {
      x: runtimeState.tfPose.x + Math.cos(a) * r,
      y: runtimeState.tfPose.y + Math.sin(a) * r,
    }
  })
  runtimeState.lastTfAt = now
  runtimeState.lastLidarAt = now
  runtimeState.lastSnapshotAt = now
}

export function newId() {
  return randomUUID()
}
