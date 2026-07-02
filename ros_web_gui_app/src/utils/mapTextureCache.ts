import * as THREE from 'three'

/** Shared cache across all rendering paths — same URL never decoded twice */
export const mapTextureCache = new Map<string, {
  texture: THREE.DataTexture
  width: number
  height: number
}>()
