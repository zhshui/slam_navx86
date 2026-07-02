import * as THREE from 'three';
import { BaseLayer } from './BaseLayer';
import type { LayerConfig } from '../../types/LayerConfig';
import type { RosbridgeConnection } from '../../utils/RosbridgeConnection';
import { TF2JS } from '../../utils/tf2js';

interface PointField {
  name?: string;
  offset?: number;
  datatype?: number;
  count?: number;
}

interface PointCloud2 {
  header: { frame_id: string };
  height?: number;
  width?: number;
  fields?: PointField[];
  is_bigendian?: boolean;
  point_step?: number;
  row_step?: number;
  data?: number[] | string;
  is_dense?: boolean;
}

const FLOAT32 = 7;
const FLOAT64 = 8;
const INT8 = 1;
const UINT8 = 2;
const INT16 = 3;
const UINT16 = 4;
const INT32 = 5;
const UINT32 = 6;

export class PointCloud2Layer extends BaseLayer {
  private pointMesh: THREE.Points | null = null;
  private pointColor: number;
  private pointSize: number;
  private tf2js: TF2JS;
  private targetFrame: string;
  private maxPoints: number;
  /** 是否累加点云（总地图模式） */
  private accumulate: boolean;
  /** 累加模式下最大总点数 */
  private maxTotalPoints: number;
  /** 累加缓冲: [x,y,z, x,y,z, ...] */
  private accBuffer: Float32Array | null = null;
  /** 累加颜色缓冲: [r,g,b, r,g,b, ...] */
  private accColorBuffer: Float32Array | null = null;
  /** 累加缓冲中当前有效点数 */
  private accCount = 0;
  /** 强度→颜色映射 */
  private colorByIntensity = true;
  /** 点云显示距离上限 (m)，超出不显示 */
  private maxDistance: number = Infinity;

  private updateCount = 0;
  private lastMeshUpdate = 0;
  private meshDirty = false;
  private flushTimeout: ReturnType<typeof setTimeout> | null = null;

  constructor(scene: THREE.Scene, config: LayerConfig, connection: RosbridgeConnection | null = null) {
    super(scene, config, connection);
    this.tf2js = TF2JS.getInstance();
    this.targetFrame = (config.targetFrame as string | undefined) || 'map';
    this.pointColor = (config.color as number | undefined) || 0xff6600;
    this.pointSize = (config.pointSize as number | undefined) ?? 0.05;
    this.maxPoints = (config.maxPoints as number | undefined) ?? Infinity;
    this.accumulate = (config.accumulate as boolean | undefined) ?? false;
    this.maxTotalPoints = (config.maxTotalPoints as number | undefined) ?? 500000;
    if (config.topic) {
      this.subscribe(config.topic, this.getMessageType());
    }
  }

  getMessageType(): string | null {
    return 'sensor_msgs/PointCloud2';
  }

  private readFieldValue(view: DataView, offset: number, datatype: number, littleEndian: boolean): number {
    switch (datatype) {
      case INT8: return view.getInt8(offset);
      case UINT8: return view.getUint8(offset);
      case INT16: return view.getInt16(offset, littleEndian);
      case UINT16: return view.getUint16(offset, littleEndian);
      case INT32: return view.getInt32(offset, littleEndian);
      case UINT32: return view.getUint32(offset, littleEndian);
      case FLOAT32: return view.getFloat32(offset, littleEndian);
      case FLOAT64: return view.getFloat64(offset, littleEndian);
      default: return view.getFloat32(offset, littleEndian);
    }
  }

  update(message: unknown): void {
    const msg = message as PointCloud2;
    if (!msg || !msg.fields || !msg.data) return;

    const fields = msg.fields as PointField[];
    const xField = fields.find(f => f.name === 'x');
    const yField = fields.find(f => f.name === 'y');
    const zField = fields.find(f => f.name === 'z');

    if (!xField || !yField) return;

    const pointStep = Number(msg.point_step ?? 0);
    if (pointStep <= 0) return;

    const littleEndian = !msg.is_bigendian;
    const totalPoints = Number(msg.width ?? 0) * Number(msg.height ?? 1);
    const limit = Math.min(totalPoints, this.maxPoints);

    let bytes: Uint8Array;
    if (msg.data instanceof Uint8Array) {
      bytes = msg.data;
    } else if (Array.isArray(msg.data)) {
      bytes = Uint8Array.from(msg.data as number[]);
    } else if (typeof msg.data === 'string') {
      try {
        const binary = atob(msg.data);
        bytes = new Uint8Array(binary.length);
        for (let i = 0; i < binary.length; i++) {
          bytes[i] = binary.charCodeAt(i);
        }
      } catch {
        return;
      }
    } else {
      return;
    }

    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const xOffset = Number(xField.offset ?? 0);
    const yOffset = Number(yField.offset ?? 0);
    const zOffset = zField ? Number(zField.offset ?? 0) : 0;
    const xType = Number(xField.datatype ?? FLOAT32);
    const yType = Number(yField.datatype ?? FLOAT32);
    const zType = zField ? Number(zField.datatype ?? FLOAT32) : FLOAT32;

    // 读强度和颜色字段
    const intensityField = fields.find(f => f.name === 'intensity');
    const iOff = intensityField ? Number(intensityField.offset ?? 0) : -1;
    const iType = intensityField ? Number(intensityField.datatype ?? FLOAT32) : FLOAT32;

    const posArr: number[] = []; // 扁平坐标 [x,y,z, x,y,z, ...]
    const colArr: number[] = []; // 扁平颜色 [r,g,b, r,g,b, ...]
    let minI = Infinity, maxI = -Infinity;
    const intensities: number[] = [];

    for (let i = 0; i < limit; i++) {
      const base = i * pointStep;
      if (base + pointStep > view.byteLength) break;

      const x = this.readFieldValue(view, base + xOffset, xType, littleEndian);
      const y = this.readFieldValue(view, base + yOffset, yType, littleEndian);
      const z = zField ? this.readFieldValue(view, base + zOffset, zType, littleEndian) : 0;

      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;

      // 距离过滤
      if (this.maxDistance < Infinity) {
        const dist = Math.sqrt(x * x + y * y + z * z);
        if (dist > this.maxDistance) continue;
      }

      posArr.push(x, y, z + 0.5);

      if (this.colorByIntensity && iOff >= 0) {
        const intensity = this.readFieldValue(view, base + iOff, iType, littleEndian);
        intensities.push(Number.isFinite(intensity) ? intensity : 0);
        if (intensity < minI) minI = intensity;
        if (intensity > maxI) maxI = intensity;
      }
    }

    const pointCount = posArr.length / 3;
    if (pointCount === 0) return;

    // 强度 → 颜色映射 (HSV: 蓝0→青→绿→黄→红240)
    const iRange = maxI - minI || 1;
    for (let i = 0; i < pointCount; i++) {
      if (this.colorByIntensity && iOff >= 0 && i < intensities.length) {
        const t = Math.max(0, Math.min(1, (intensities[i]! - minI) / iRange));
        const h = (1 - t) * 0.66; // 蓝(0.66)→红(0)
        const s = 1, l = 0.5;
        const c = new THREE.Color().setHSL(h, s, l);
        colArr.push(c.r, c.g, c.b);
      } else {
        const c = new THREE.Color(this.pointColor);
        colArr.push(c.r, c.g, c.b);
      }
    }

    // TF 变换
    const points3: THREE.Vector3[] = [];
    for (let i = 0; i < pointCount; i++) {
      points3.push(new THREE.Vector3(posArr[i * 3]!, posArr[i * 3 + 1]!, posArr[i * 3 + 2]!));
    }

    const sourceFrame = msg.header?.frame_id || '';
    if (sourceFrame && sourceFrame !== this.targetFrame) {
      const transformMatrix = this.tf2js.getTransformMatrix(sourceFrame, this.targetFrame);
      if (transformMatrix) {
        for (let i = 0; i < pointCount; i++) {
          points3[i]!.applyMatrix4(transformMatrix);
          posArr[i * 3] = points3[i]!.x;
          posArr[i * 3 + 1] = points3[i]!.y;
          posArr[i * 3 + 2] = points3[i]!.z;
        }
      }
    }

    if (this.accumulate) {
      this.accumulatePoints(posArr, colArr, pointCount);
    } else {
      this.replacePoints(posArr, colArr, pointCount);
    }

    this.updateCount++;
    if (this.updateCount <= 2 || this.updateCount % 30 === 0) {
      const total = this.accumulate ? this.accCount : pointCount;
      console.log(`[PointCloud2Layer] #${this.updateCount} ${this.accumulate ? '累加' : '替换'} 新增${pointCount} 总计${total} 强度范围[${minI.toFixed(0)},${maxI.toFixed(0)}] 帧=${sourceFrame}`);
    }
  }

  /** 累加模式：滑动窗口 */
  private accumulatePoints(posArr: number[], colArr: number[], pointCount: number): void {
    if (pointCount === 0) return;

    if (!this.accBuffer) {
      this.accBuffer = new Float32Array(this.maxTotalPoints * 3);
      this.accColorBuffer = new Float32Array(this.maxTotalPoints * 3);
    }

    const overflow = this.accCount + pointCount - this.maxTotalPoints;
    if (overflow > 0) {
      const keepCount = this.accCount - overflow;
      if (keepCount > 0) {
        this.accBuffer.copyWithin(0, overflow * 3, this.accCount * 3);
        this.accColorBuffer!.copyWithin(0, overflow * 3, this.accCount * 3);
      }
      this.accCount = Math.max(0, keepCount);
    }

    for (let i = 0; i < pointCount; i++) {
      const off = this.accCount * 3;
      this.accBuffer![off] = posArr[i * 3]!;
      this.accBuffer![off + 1] = posArr[i * 3 + 1]!;
      this.accBuffer![off + 2] = posArr[i * 3 + 2]!;
      if (colArr.length > 0) {
        this.accColorBuffer![off] = colArr[i * 3]!;
        this.accColorBuffer![off + 1] = colArr[i * 3 + 1]!;
        this.accColorBuffer![off + 2] = colArr[i * 3 + 2]!;
      }
      this.accCount++;
    }

    this._updateOrCreateMesh(this.accBuffer, this.accColorBuffer, this.accCount);
  }

  /** 替换模式 */
  private replacePoints(posArr: number[], colArr: number[], pointCount: number): void {
    const posBuf = new Float32Array(posArr);
    const colBuf = colArr.length > 0 ? new Float32Array(colArr) : null;
    this._updateOrCreateMesh(posBuf, colBuf, pointCount);
  }

  private _flushMesh(): void {
    if (!this.meshDirty || !this.accBuffer || this.accCount === 0) return;
    this.meshDirty = false;
    this.lastMeshUpdate = performance.now();

    const count = this.accCount;
    const posData = this.accBuffer.slice(0, count * 3);
    const colData = this.accColorBuffer ? this.accColorBuffer.slice(0, count * 3) : null;

    if (this.pointMesh) {
      const geom = this.pointMesh.geometry;
      const posAttr = geom.attributes.position as THREE.BufferAttribute;
      const colAttr = geom.attributes.color as THREE.BufferAttribute | undefined;

      // Reuse or create position attribute
      if (posAttr && posAttr.count >= count) {
        posAttr.copyArray(posData);
        posAttr.count = count;
        posAttr.needsUpdate = true;
      } else {
        geom.setAttribute('position', new THREE.BufferAttribute(posData, 3));
      }

      // Reuse or create color attribute
      if (colData) {
        if (colAttr && colAttr.count >= count) {
          colAttr.copyArray(colData);
          colAttr.count = count;
          colAttr.needsUpdate = true;
        } else {
          geom.setAttribute('color', new THREE.BufferAttribute(colData, 3));
        }
      }

      geom.setDrawRange(0, count);
    } else {
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.BufferAttribute(posData, 3));
      if (colData) {
        geom.setAttribute('color', new THREE.BufferAttribute(colData, 3));
      }
      const hasColors = !!colData;
      const mat = new THREE.PointsMaterial({
        color: hasColors ? 0xffffff : this.pointColor,
        size: this.pointSize,
        sizeAttenuation: true,
        depthTest: false,
        depthWrite: false,
        transparent: true,
        vertexColors: hasColors,
      });
      this.pointMesh = new THREE.Points(geom, mat);
      this.pointMesh.renderOrder = 9999;
      this.object3D = this.pointMesh;
      this.scene.add(this.pointMesh);
    }
  }

  private _updateOrCreateMesh(posBuf: Float32Array, colBuf: Float32Array | null, count: number): void {
    // 累加模式：标记脏数据，节流更新 (最多 300ms 一次)
    if (this.accumulate) {
      this.meshDirty = true;
      const now = performance.now();
      if (now - this.lastMeshUpdate > 300) {
        this._flushMesh();
      } else if (!this.flushTimeout) {
        // 无新数据时 500ms 后强制刷新，不留尾帧
        this.flushTimeout = setTimeout(() => {
          this.flushTimeout = null;
          if (this.meshDirty) this._flushMesh();
        }, 500);
      }
      return;
    }

    // 替换模式：直接更新
    if (this.pointMesh) {
      const geom = this.pointMesh.geometry;
      const posAttr = geom.attributes.position as THREE.BufferAttribute;
      const colAttr = geom.attributes.color as THREE.BufferAttribute | undefined;
      if (posAttr && posAttr.count >= count) {
        posAttr.copyArray(posBuf);
        posAttr.count = count;
        posAttr.needsUpdate = true;
      } else {
        geom.setAttribute('position', new THREE.BufferAttribute(posBuf.slice(0, count * 3), 3));
      }
      if (colBuf && colBuf.length >= count * 3) {
        if (colAttr && colAttr.count >= count) {
          colAttr.copyArray(colBuf);
          colAttr.count = count;
          colAttr.needsUpdate = true;
        } else {
          geom.setAttribute('color', new THREE.BufferAttribute(colBuf.slice(0, count * 3), 3));
        }
      }
      geom.setDrawRange(0, count);
    } else {
      const hasColors = colBuf && colBuf.length >= count * 3;
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.BufferAttribute(posBuf.slice(0, count * 3), 3));
      if (hasColors && colBuf) {
        geom.setAttribute('color', new THREE.BufferAttribute(colBuf.slice(0, count * 3), 3));
      }
      const mat = new THREE.PointsMaterial({
        color: hasColors ? 0xffffff : this.pointColor,
        size: this.pointSize,
        sizeAttenuation: true,
        depthTest: false,
        depthWrite: false,
        transparent: true,
        vertexColors: hasColors,
      });
      this.pointMesh = new THREE.Points(geom, mat);
      this.pointMesh.renderOrder = 9999;
      this.object3D = this.pointMesh;
      this.scene.add(this.pointMesh);
    }
  }

  setConfig(config: LayerConfig): void {
    super.setConfig(config);
    if (config.color !== undefined) this.pointColor = config.color as number;
    if (config.pointSize !== undefined) this.pointSize = config.pointSize as number;
    if (config.targetFrame !== undefined) this.targetFrame = config.targetFrame as string;
    if (config.maxPoints !== undefined) this.maxPoints = config.maxPoints as number;
    const newAccumulate = (config.accumulate as boolean | undefined) ?? false;
    if (newAccumulate !== this.accumulate) {
      // 模式切换时清空累加缓冲
      this.accumulate = newAccumulate;
      this.accBuffer = null;
      this.accColorBuffer = null;
      this.accCount = 0;
      if (this.pointMesh) {
        this.scene.remove(this.pointMesh);
        this.pointMesh.geometry.dispose();
        (this.pointMesh.material as THREE.Material).dispose();
        this.pointMesh = null;
        this.object3D = null;
      }
    }
    if (config.maxDistance !== undefined) this.maxDistance = config.maxDistance as number;
    if (config.maxTotalPoints !== undefined) {
      this.maxTotalPoints = config.maxTotalPoints as number;
      if (this.accumulate && this.accBuffer && this.accCount > 0) {
        this._flushMesh();
      }
    }
  }

  dispose(): void {
    if (this.pointMesh) {
      this.scene.remove(this.pointMesh);
      this.pointMesh.geometry.dispose();
      (this.pointMesh.material as THREE.Material).dispose();
      this.pointMesh = null;
    }
    if (this.flushTimeout) { clearTimeout(this.flushTimeout); this.flushTimeout = null; }
    this.accBuffer = null;
    this.accColorBuffer = null;
    this.accCount = 0;
    super.dispose();
  }
}
