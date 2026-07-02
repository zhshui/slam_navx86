export type Role = 'admin' | 'operator'

export interface User {
  id: string
  username: string
  role: Role
}

export interface OccupancyMap {
  width: number
  height: number
  resolution: number
  origin: { x: number; y: number }
  data: number[]
  /** PGM image URL for large maps — frontend loads as texture instead of using raw data array */
  mapUrl?: string | null
}

export interface NavPoint {
  id: string
  name: string
  x: number
  y: number
  yaw: number
  map_id: string | null
  created_at: string
}

export type ObstacleShape = 'rect' | 'circle' | 'polygon'

export interface VirtualObstacle {
  id: string
  name: string
  shape: ObstacleShape
  data: unknown
  enabled: number
  map_id: string | null
  created_at: string
}

export interface MapRecord {
  id: string
  name: string
  yaml_path: string
  pcd_path: string
  created_at: string
  active: number
}

export interface Snapshot {
  runtime: {
    rosConnected: boolean
    mappingStatus: string
    mappingPauseSupported: boolean
    navStatus: string
    navMode: string
    hdlRunning: boolean
    lidarStatus: string
    motionStatus: string
    currentMapId: string | null
    map: OccupancyMap
    mapUpdatedAt: string | null
    lastTfAt: string | null
    lastVoxelAt: string | null
    tfPose: { x: number; y: number; yaw: number; frame: string }
    lidar: Array<{ x: number; y: number }>
    globalPlan: Array<{ x: number; y: number }>
    localPlan: Array<{ x: number; y: number }>
    voxelGrid: Array<{ x: number; y: number }>
    lastSnapshotAt: string
  }
  navPoints: NavPoint[]
  virtualObstacles: VirtualObstacle[]
  maps: MapRecord[]
}
