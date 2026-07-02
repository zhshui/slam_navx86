import * as THREE from 'three';
import { BaseLayer } from './BaseLayer';
import type { LayerConfig } from '../../types/LayerConfig';
import type { RosbridgeConnection } from '../../utils/RosbridgeConnection';
import { TF2JS } from '../../utils/tf2js';

interface Point {
  x: number;
  y: number;
  z: number;
}

interface Pose {
  position: Point;
  orientation: { x: number; y: number; z: number; w: number };
}

interface PoseStamped {
  pose: Pose;
}

interface Path {
  header: {
    frame_id: string;
  };
  poses: PoseStamped[];
}

export class PathLayer extends BaseLayer {
  private mesh: THREE.Mesh | null = null;
  private color: number;
  private lineWidth: number;
  private tf2js: TF2JS;
  private mapFrame: string;

  constructor(scene: THREE.Scene, config: LayerConfig, connection: RosbridgeConnection | null = null) {
    super(scene, config, connection);
    this.tf2js = TF2JS.getInstance();
    this.mapFrame = (config.mapFrame as string | undefined) || 'map';
    this.color = (config.color as number | undefined) || 0x00ff00;
    this.lineWidth = (config.lineWidth as number | undefined) ?? 1;
    if (config.topic) {
      this.subscribe(config.topic, this.getMessageType());
    }
  }

  getMessageType(): string | null {
    return 'nav_msgs/Path';
  }

  update(message: unknown): void {
    const msg = message as Path;
    if (!msg.poses || !Array.isArray(msg.poses) || msg.poses.length === 0) {
      if (this.mesh) {
        this.scene.remove(this.mesh);
        this.mesh.geometry.dispose();
        (this.mesh.material as THREE.Material).dispose();
        this.mesh = null;
        this.object3D = null;
      }
      return;
    }

    const sourceFrame = msg.header?.frame_id || '';

    if (this.mesh) {
      this.scene.remove(this.mesh);
      this.mesh.geometry.dispose();
      (this.mesh.material as THREE.Material).dispose();
      this.mesh = null;
    }

    const pointData = msg.poses.map(poseStamped => ({
      x: poseStamped.pose.position.x,
      y: poseStamped.pose.position.y,
      z: poseStamped.pose.position.z + 0.01
    }));

    const transformedPoints = this.tf2js.transformPointsToFrame(pointData, sourceFrame, this.mapFrame);
    if (!transformedPoints) {
      console.warn('[PathLayer] Transform not found:', { sourceFrame, targetFrame: this.mapFrame });
      return;
    }

    if (transformedPoints.length >= 2) {
      // 使用 TubeGeometry 渲染粗线
      const curve = new THREE.CatmullRomCurve3(transformedPoints, false, 'catmullrom', 0.5);
      const radius = Math.max(0.02, this.lineWidth * 0.02); // lineWidth=5 → radius=0.1
      const tubeGeom = new THREE.TubeGeometry(curve, transformedPoints.length * 2, radius, 8, false);
      const tubeMat = new THREE.MeshBasicMaterial({
        color: this.color,
        depthTest: false,
        depthWrite: false,
        transparent: true,
      });
      const tube = new THREE.Mesh(tubeGeom, tubeMat);
      tube.renderOrder = 5000;
      this.mesh = tube;
      this.object3D = tube;
      this.scene.add(tube);
    }
  }

  setConfig(config: LayerConfig): void {
    const cfg = config as LayerConfig & { mapFrame?: string };
    if (cfg.mapFrame) this.mapFrame = cfg.mapFrame;
    this.color = (config.color as number | undefined) ?? this.color;
    this.lineWidth = (config.lineWidth as number | undefined) ?? this.lineWidth;
    super.setConfig(config);
  }

  dispose(): void {
    if (this.mesh) {
      this.scene.remove(this.mesh);
      this.mesh.geometry.dispose();
      (this.mesh.material as THREE.Material).dispose();
      this.mesh = null;
    }
    super.dispose();
  }
}

