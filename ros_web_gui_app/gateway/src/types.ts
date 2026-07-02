export type Role = 'admin' | 'operator'

export interface User {
  id: string
  username: string
  role: Role
  password_hash: string
  created_at: string
}

export interface MapRecord {
  id: string
  name: string
  yaml_path: string
  created_at: string
  active: number
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
  data: string
  enabled: number
  map_id: string | null
  created_at: string
}
