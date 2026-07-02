import * as THREE from 'three';
import { BaseLayer } from './BaseLayer';
import type { LayerConfig } from '../../types/LayerConfig';
import type { RosbridgeConnection } from '../../utils/RosbridgeConnection';
import {
  paletteColorCached,
  rgbaToCssString,
} from '../../utils/colorUtils';
import type { ColorModes } from '../../utils/colorUtils';
import { MapManager, type OccupancyGrid } from '../../utils/MapManager';
import { TF2JS } from '../../utils/tf2js';
import { getSafeMaxTextureSize, downsampleOccupancyGrid } from '../../utils/mapDownsample';
import { mapTextureCache } from '../../utils/mapTextureCache';

/**
 * Load a grayscale PNG map, downsample at canvas-draw time, convert to DataTexture.
 * Caches by URL base — same map won't be decoded twice.
 */
function loadPngMapTexture(
  url: string, origWidth: number, origHeight: number, lut: Uint8Array, maxTexSize: number
): Promise<{ texture: THREE.DataTexture; width: number; height: number }> {
  const cacheKey = url  // include ?t= for publish refresh
  const cached = mapTextureCache.get(cacheKey)
  if (cached) return Promise.resolve(cached)
  return new Promise((resolve, reject) => {
    const factor = Math.max(1, Math.ceil(Math.max(origWidth, origHeight) / maxTexSize));
    const dw = Math.floor(origWidth / factor);
    const dh = Math.floor(origHeight / factor);

    const img = new Image();
    img.crossOrigin = 'anonymous';
    img.onload = () => {
      const canvas = document.createElement('canvas');
      canvas.width = dw;
      canvas.height = dh;
      const ctx = canvas.getContext('2d');
      if (!ctx) { reject(new Error('No 2d context')); return; }
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, 0, 0, origWidth, origHeight, 0, 0, dw, dh);
      const imageData = ctx.getImageData(0, 0, dw, dh);
      // Flip Y: canvas row 0 (top) → rg barow (dh-1) (bottom)
      const rgba = new Uint8ClampedArray(dw * dh * 4);
      for (let cy = 0; cy < dh; cy++) {
        const srcRow = cy * dw * 4;
        const dstRow = (dh - 1 - cy) * dw * 4;
        for (let cx = 0; cx < dw; cx++) {
          const gray = imageData.data[srcRow + cx * 4]!;
          const dst = dstRow + cx * 4;
          let src: number;
          if (gray <= 1) src = 100 * 4;
          else if (gray >= 250) src = 0;
          else src = 255 * 4;
          rgba[dst] = lut[src]!;
          rgba[dst + 1] = lut[src + 1]!;
          rgba[dst + 2] = lut[src + 2]!;
          rgba[dst + 3] = lut[src + 3]!;
        }
      }
      const texture = new THREE.DataTexture(
        rgba, dw, dh, THREE.RGBAFormat,
        THREE.UnsignedByteType, THREE.UVMapping,
        THREE.ClampToEdgeWrapping, THREE.ClampToEdgeWrapping,
        THREE.NearestFilter, THREE.LinearFilter, 1,
        THREE.LinearSRGBColorSpace,
      );
      texture.generateMipmaps = false;
      texture.flipY = false;
      texture.needsUpdate = true;
      const result = { texture, width: dw, height: dh };
      mapTextureCache.set(cacheKey, result);
      resolve(result);
    };
    img.onerror = () => reject(new Error('Map PNG load failed: ' + url));
    img.src = url;
  });
}


interface OccupancyGridSettings {
  colorMode?: ColorModes;
  minColor?: string;
  maxColor?: string;
  unknownColor?: string;
  invalidColor?: string;
  alpha?: number;
  height?: number;
}

const DEFAULT_MIN_COLOR = { r: 1, g: 1, b: 1, a: 1 };
const DEFAULT_MAX_COLOR = { r: 0, g: 0, b: 0, a: 1 };
const DEFAULT_UNKNOWN_COLOR = { r: 0.5, g: 0.5, b: 0.5, a: 1 };
const DEFAULT_INVALID_COLOR = { r: 1, g: 0, b: 1, a: 1 };

/** Pre-compute a lookup table for fast occ→rgba conversion */
function buildOccToRgbaLut(colorMode: ColorModes, alpha: number): Uint8Array {
  const lut = new Uint8Array(256 * 4);
  const tempColor = { r: 0, g: 0, b: 0, a: 0 };
  for (let v = 0; v <= 100; v++) {
    const offset = v * 4;
    paletteColorCached(tempColor, v, colorMode);
    lut[offset] = tempColor.r;
    lut[offset + 1] = tempColor.g;
    lut[offset + 2] = tempColor.b;
    lut[offset + 3] = Math.trunc(tempColor.a * alpha);
  }
  // unknown (-1) → transparent
  lut[255 * 4] = 0;
  lut[255 * 4 + 1] = 0;
  lut[255 * 4 + 2] = 0;
  lut[255 * 4 + 3] = 0;
  return lut;
}

export class OccupancyGridLayer extends BaseLayer {
  private mesh: THREE.Mesh | null = null;
  protected texture: THREE.DataTexture | null = null;
  private settings: OccupancyGridSettings;
  protected lastData: number[] | Int8Array | null = null;
  protected lastWidth: number = 0;
  protected lastHeight: number = 0;
  /** Original (un-downsampled) dimensions for coordinate mapping */
  protected origWidth: number = 0;
  protected origHeight: number = 0;
  protected lastMessage: OccupancyGrid | null = null;
  protected mapManager: MapManager;
  private handleMapUpdate: ((map: OccupancyGrid | null) => void) | null = null;
  private tf2js: TF2JS;
  private mapFrame: string;
  private occLut: Uint8Array | null = null;

  constructor(scene: THREE.Scene, config: LayerConfig, connection: RosbridgeConnection | null = null) {
    super(scene, config, connection);
    this.tf2js = TF2JS.getInstance();
    this.mapFrame = (config.mapFrame as string | undefined) || 'map';
    this.settings = {
      colorMode: (config.colorMode as ColorModes | undefined) || 'map',
      minColor: (config.minColor as string | undefined) || rgbaToCssString(DEFAULT_MIN_COLOR),
      maxColor: (config.maxColor as string | undefined) || rgbaToCssString(DEFAULT_MAX_COLOR),
      unknownColor: (config.unknownColor as string | undefined) || rgbaToCssString(DEFAULT_UNKNOWN_COLOR),
      invalidColor: (config.invalidColor as string | undefined) || rgbaToCssString(DEFAULT_INVALID_COLOR),
      alpha: (config.alpha as number | undefined) ?? 1.0,
      height: (config.height as number | undefined) ?? 0,
    };
    this.occLut = buildOccToRgbaLut(this.settings.colorMode!, this.settings.alpha!);
    this.mapManager = MapManager.getInstance();

    if (config.topic === '/map') {
      this.handleMapUpdate = (map: OccupancyGrid | null) => {
        if (map && this.config.enabled) {
          this.renderMap(map);
        }
      };
      this.mapManager.addOccupancyGridListener(this.handleMapUpdate);
      const currentMap = this.mapManager.getOccupancyGrid();
      if (currentMap && this.config.enabled) {
        this.renderMap(currentMap);
      }
    } else {
      if (config.topic) {
        this.subscribe(config.topic, this.getMessageType());
      }
    }
  }

  private renderMapGen = 0;

  renderMap(msg: OccupancyGrid): void {
    if (!msg.info) return;

    let width = msg.info.width;
    let height = msg.info.height;
    const resolution = msg.info.resolution;
    const origin = msg.info.origin;
    const size = width * height;

    const msgAny = msg as any;
    const mapUrl: string | undefined = msgAny.mapUrl;
    const hasDirectData = msg.data && msg.data.length === size;

    // Need either raw data or a PGM URL to render
    if (!hasDirectData && !mapUrl) return;

    const gen = ++this.renderMapGen;

    // Downsample if map exceeds mobile GPU max texture size
    // Keep original data for editing; only downsample the texture copy
    const maxTexSize = getSafeMaxTextureSize();
    let mapData: number[] | Int8Array | null = hasDirectData ? msg.data : null;
    const originalData = mapData; // keep reference for editing
    let scaleCompensation = 1;
    if (mapData && (width > maxTexSize || height > maxTexSize)) {
      const result = downsampleOccupancyGrid(width, height, mapData, maxTexSize);
      scaleCompensation = width / result.width; // adjust resolution compensation
      console.log(
        `[OccupancyGridLayer] Downsampled map from ${width}x${height} to ${result.width}x${result.height} ` +
        `(max texture size: ${maxTexSize})`
      );
      width = result.width;
      height = result.height;
      mapData = result.data;
    }

    // Ensure mesh/texture exist (or re-create if size changed)
    if (!this.mesh) {
      const geometry = new THREE.PlaneGeometry(1, 1, 1, 1);
      geometry.translate(0.5, 0.5, 0);
      const texture = this.createTexture(width, height);
      const material = this.createMaterial(texture);
      const mesh = new THREE.Mesh(geometry, material);
      this.mesh = mesh;
      this.texture = texture;
      this.object3D = mesh;
      mesh.renderOrder = -1;
      this.scene.add(mesh);
    }

    if (this.texture && (width !== this.texture.image.width || height !== this.texture.image.height)) {
      this.texture.dispose();
      this.texture = this.createTexture(width, height);
      (this.mesh.material as THREE.MeshBasicMaterial).map = this.texture;
    }

    const doPosition = () => {
      // Store effective resolution so editor tools map world→grid correctly
      const effectiveResolution = resolution * scaleCompensation;
      if (originalData) {
        // Use ORIGINAL full-resolution data for editing (not the downsampled copy)
        this.lastData = Array.isArray(originalData) ? new Int8Array(originalData) : new Int8Array(originalData as ArrayLike<number>);
        this.lastWidth = msg.info.width;
        this.lastHeight = msg.info.height;
        this.origWidth = msg.info.width;
        this.origHeight = msg.info.height;
        this.lastMessage = {
          header: msg.header ? { ...msg.header } : { frame_id: '' },
          info: { ...msg.info, width: msg.info.width, height: msg.info.height, resolution: msg.info.resolution },
          data: this.lastData,
        };
      } else {
        // Large map via URL — no cell-level editing until PGM is loaded
        this.lastWidth = width;
        this.lastHeight = height;
        this.lastMessage = {
          header: msg.header ? { ...msg.header } : { frame_id: '' },
          info: { ...msg.info, width, height, resolution: effectiveResolution },
          data: new Int8Array(0),
        };
      }

      const mapWidth = width * resolution * scaleCompensation;
      const mapHeight = height * resolution * scaleCompensation;
      this.mesh!.scale.set(mapWidth, mapHeight, 1);

      const sourceFrame = msg.header?.frame_id || '';
      const originPosition = new THREE.Vector3(origin.position.x, origin.position.y, origin.position.z);
      const originQuaternion = new THREE.Quaternion(
        origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w
      );

      if (sourceFrame) {
        const transform = this.tf2js.findTransform(this.mapFrame, sourceFrame);
        if (transform) {
          const transformMatrix = new THREE.Matrix4();
          transformMatrix.makeRotationFromQuaternion(transform.rotation);
          transformMatrix.setPosition(transform.translation);
          originPosition.applyMatrix4(transformMatrix);
          originQuaternion.premultiply(transform.rotation);
        }
      }

      this.mesh!.position.set(
        originPosition.x,
        originPosition.y,
        originPosition.z + (this.settings.height ?? 0)
      );
      this.mesh!.quaternion.copy(originQuaternion);
    };

    if (mapData) {
      this.updateTextureFull(this.texture!, mapData, width, height);
      doPosition();
    } else if (mapUrl) {
      // Large map — load PNG with built-in downsampling at load time
      const loadGen = gen;
      const maxTexSize = getSafeMaxTextureSize();
      loadPngMapTexture(mapUrl, width, height, this.occLut!, maxTexSize)
        .then(({ texture: tex, width: newW, height: newH }) => {
          // Skip if a more recent renderMap call already loaded data (e.g. from PGM editor fetch)
          if (loadGen !== this.renderMapGen) return;
          if (this.texture) { this.texture.dispose(); }
          this.texture = tex;
          (this.mesh!.material as THREE.MeshBasicMaterial).map = tex;
          scaleCompensation = width / newW;
          width = newW;
          height = newH;
          doPosition();
        })
        .catch((err) => console.error('[OccupancyGridLayer] Map PNG load failed:', err));
    }
  }

  getMessageType(): string | null {
    return 'nav_msgs/OccupancyGrid';
  }

  setConnection(connection: RosbridgeConnection): void {
    this.connection = connection;
    if (this.config.topic === '/map') {
      return;
    }
    if (this.config.topic && connection.isConnected()) {
      this.subscribe(this.config.topic, this.getMessageType());
    }
  }

  update(message: unknown): void {
    const msg = message as OccupancyGrid;
    if (!msg.info || !msg.data) return;
    if (this.config.topic === '/map') {
      return;
    }
    this.renderMap(msg);
  }

  private createTexture(width: number, height: number): THREE.DataTexture {
    const size = width * height;
    const rgba = new Uint8ClampedArray(size * 4);
    const texture = new THREE.DataTexture(
      rgba,
      width,
      height,
      THREE.RGBAFormat,
      THREE.UnsignedByteType,
      THREE.UVMapping,
      THREE.ClampToEdgeWrapping,
      THREE.ClampToEdgeWrapping,
      THREE.NearestFilter,
      THREE.LinearFilter,
      1,
      THREE.LinearSRGBColorSpace,
    );
    texture.generateMipmaps = false;
    // Data is in ROS OccupancyGrid convention (row 0 = bottom/origin).
    // flipY=false ensures Three.js doesn't invert the rows.
    texture.flipY = false;
    return texture;
  }

  private createMaterial(texture: THREE.DataTexture): THREE.MeshBasicMaterial {
    const transparent = this.settings.alpha! < 1.0;
    return new THREE.MeshBasicMaterial({
      map: texture,
      side: THREE.DoubleSide,
      alphaTest: 1e-4,
      depthWrite: false,
      depthTest: false,
      transparent,
      opacity: this.settings.alpha,
    });
  }

  /** Full texture rebuild — only used on initial load / topic change */
  private updateTextureFull(texture: THREE.DataTexture, data: number[] | Int8Array, width: number, height: number): void {
    const rgba = texture.image.data as Uint8ClampedArray;
    const lut = this.occLut!;
    const size = width * height;

    for (let i = 0; i < size; i++) {
      const val = data[i]!;
      if (val < 0) {
        // unknown → transparent
        rgba[i * 4 + 3] = 0;
      } else {
        const src = Math.min(val, 100) * 4; // clamp to LUT range
        const dst = i * 4;
        rgba[dst] = lut[src]!;
        rgba[dst + 1] = lut[src + 1]!;
        rgba[dst + 2] = lut[src + 2]!;
        rgba[dst + 3] = lut[src + 3]!;
      }
    }
    texture.needsUpdate = true;
  }

  private updateTexturePartial(indices: number[], values: number[]): void {
    if (!this.texture || indices.length === 0) return;
    const rgba = this.texture.image.data as Uint8ClampedArray;
    const lut = this.occLut!;
    const texW = this.texture.image.width;
    const texH = this.texture.image.height;
    const factor = this.origWidth > 0 ? this.origWidth / texW : 1;

    // For large maps: use a Uint8Array dirty mask instead of Set for O(1) dedup
    const dirty = factor > 1 ? new Uint8Array(texW * texH) : null;

    for (let j = 0; j < indices.length; j++) {
      const origIdx = indices[j]!;
      const val = values[j]!;
      let dst: number;
      if (dirty) {
        const oy = (origIdx / this.origWidth) | 0;
        const ox = origIdx - oy * this.origWidth;
        const tx = (ox / factor) | 0;
        const ty = (oy / factor) | 0;
        const ti = ty * texW + tx;
        if (dirty[ti]) continue;
        dirty[ti] = 1;
        dst = ti * 4;
      } else {
        dst = origIdx * 4;
      }
      if (val < 0) {
        rgba[dst + 3] = 0;
      } else {
        const src = Math.min(val, 100) * 4;
        rgba[dst] = lut[src]!; rgba[dst+1] = lut[src+1]!; rgba[dst+2] = lut[src+2]!; rgba[dst+3] = lut[src+3]!;
      }
    }
    this.texture.needsUpdate = true;
  }

  modifyCell(worldX: number, worldY: number, value: number): boolean {
    if (!this.lastMessage || !this.lastData || !this.mesh) return false;

    const resolution = this.lastMessage.info.resolution;
    const origin = this.lastMessage.info.origin;
    const width = this.origWidth || this.lastWidth;
    const height = this.origHeight || this.lastHeight;

    const gridX = Math.floor((worldX - origin.position.x) / resolution);
    const gridY = Math.floor((worldY - origin.position.y) / resolution);

    if (gridX < 0 || gridX >= width || gridY < 0 || gridY >= height) return false;

    const index = gridY * width + gridX;
    if (index >= 0 && index < this.lastData.length) {
      this.lastData[index] = value;
      this.updateTexturePartial([index], [value]);
      return true;
    }
    return false;
  }

  modifyCells(
    worldPositions: Array<{ x: number; y: number }>,
    value: number,
    brushSize: number = 1,
    initialValues?: Map<number, number>
  ): Array<{ index: number; oldValue: number; newValue: number }> {
    if (!this.lastMessage || !this.lastData || !this.mesh) return [];

    const resolution = this.lastMessage.info.resolution;
    const origin = this.lastMessage.info.origin;
    const width = this.origWidth || this.lastWidth;
    const height = this.origHeight || this.lastHeight;

    const changes: Array<{ index: number; oldValue: number; newValue: number }> = [];
    const modifiedIndices = new Set<number>();
    const changedIdxList: number[] = [];
    const changedValList: number[] = [];

    const radius = Math.ceil(brushSize / resolution / 2);

    for (const worldPos of worldPositions) {
      const centerGridX = Math.floor((worldPos.x - origin.position.x) / resolution);
      const centerGridY = Math.floor((worldPos.y - origin.position.y) / resolution);

      for (let dy = -radius; dy <= radius; dy++) {
        for (let dx = -radius; dx <= radius; dx++) {
          const gridX = centerGridX + dx;
          const gridY = centerGridY + dy;

          if (gridX >= 0 && gridX < width && gridY >= 0 && gridY < height) {
            const dist = Math.sqrt(dx * dx + dy * dy) * resolution;
            if (dist <= brushSize / 2) {
              const index = gridY * width + gridX;
              if (!modifiedIndices.has(index)) {
                modifiedIndices.add(index);
                const oldValue = initialValues?.get(index) ?? this.lastData[index]!;
                changes.push({ index, oldValue, newValue: value });
                this.lastData[index] = value;
                changedIdxList.push(index);
                changedValList.push(value);
              }
            }
          }
        }
      }
    }

    // Only update changed pixels — not the whole texture
    if (changedIdxList.length > 0) {
      this.updateTexturePartial(changedIdxList, changedValList);
    }

    return changes;
  }

  drawLine(startX: number, startY: number, endX: number, endY: number, value: number, lineWidth: number = 0.05): Array<{ index: number; oldValue: number; newValue: number }> {
    if (!this.lastMessage || !this.lastData || !this.mesh) return [];

    const dx = endX - startX;
    const dy = endY - startY;
    const length = Math.sqrt(dx * dx + dy * dy);
    if (length === 0) return [];

    const resolution = this.lastMessage.info.resolution;
    const stepSize = resolution;
    const steps = Math.ceil(length / stepSize);
    const positions: Array<{ x: number; y: number }> = [];

    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      positions.push({
        x: startX + dx * t,
        y: startY + dy * t,
      });
    }

    return this.modifyCells(positions, value, lineWidth);
  }

  drawRect(x1: number, y1: number, x2: number, y2: number, value: number): Array<{ index: number; oldValue: number; newValue: number }> {
    if (!this.lastMessage || !this.lastData || !this.mesh) return [];

    const resolution = this.lastMessage.info.resolution;
    const origin = this.lastMessage.info.origin;
    const width = this.origWidth || this.lastWidth;
    const height = this.origHeight || this.lastHeight;

    const minX = Math.min(x1, x2);
    const maxX = Math.max(x1, x2);
    const minY = Math.min(y1, y2);
    const maxY = Math.max(y1, y2);

    const gridMinX = Math.floor((minX - origin.position.x) / resolution);
    const gridMaxX = Math.floor((maxX - origin.position.x) / resolution);
    const gridMinY = Math.floor((minY - origin.position.y) / resolution);
    const gridMaxY = Math.floor((maxY - origin.position.y) / resolution);

    const changes: Array<{ index: number; oldValue: number; newValue: number }> = [];
    const changedIdxList: number[] = [];
    const changedValList: number[] = [];

    for (let gy = gridMinY; gy <= gridMaxY; gy++) {
      for (let gx = gridMinX; gx <= gridMaxX; gx++) {
        if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
          const index = gy * width + gx;
          const oldValue = this.lastData[index]!;
          if (oldValue !== value) {
            changes.push({ index, oldValue, newValue: value });
            this.lastData[index] = value;
            changedIdxList.push(index);
            changedValList.push(value);
          }
        }
      }
    }

    if (changedIdxList.length > 0) {
      this.updateTexturePartial(changedIdxList, changedValList);
    }

    return changes;
  }

  drawCircle(cx: number, cy: number, radius: number, value: number): Array<{ index: number; oldValue: number; newValue: number }> {
    if (!this.lastMessage || !this.lastData || !this.mesh) return [];

    const resolution = this.lastMessage.info.resolution;
    const origin = this.lastMessage.info.origin;
    const w = this.origWidth || this.lastWidth;
    const h = this.origHeight || this.lastHeight;

    const centerGridX = Math.floor((cx - origin.position.x) / resolution);
    const centerGridY = Math.floor((cy - origin.position.y) / resolution);
    const gridRadius = Math.ceil(radius / resolution);

    const changes: Array<{ index: number; oldValue: number; newValue: number }> = [];
    const changedIdxList: number[] = [];
    const changedValList: number[] = [];

    for (let dy = -gridRadius; dy <= gridRadius; dy++) {
      for (let dx = -gridRadius; dx <= gridRadius; dx++) {
        const gx = centerGridX + dx;
        const gy = centerGridY + dy;
        if (gx >= 0 && gx < w && gy >= 0 && gy < h) {
          const dist = Math.sqrt(dx * dx + dy * dy) * resolution;
          if (dist <= radius) {
            const index = gy * w + gx;
            const oldValue = this.lastData[index]!;
            if (oldValue !== value) {
              changes.push({ index, oldValue, newValue: value });
              this.lastData[index] = value;
              changedIdxList.push(index);
              changedValList.push(value);
            }
          }
        }
      }
    }

    if (changedIdxList.length > 0) {
      this.updateTexturePartial(changedIdxList, changedValList);
    }

    return changes;
  }

  getMapMessage(): OccupancyGrid | null {
    if (this.lastMessage) return this.lastMessage;
    return this.mapManager.getOccupancyGrid();
  }

  dispose(): void {
    if (this.handleMapUpdate) {
      this.mapManager.removeOccupancyGridListener(this.handleMapUpdate);
      this.handleMapUpdate = null;
    }
    if (this.texture) {
      this.texture.dispose();
      this.texture = null;
    }
    if (this.mesh) {
      this.scene.remove(this.mesh);
      this.mesh.geometry.dispose();
      (this.mesh.material as THREE.Material).dispose();
      this.mesh = null;
    }
    super.dispose();
  }
}
