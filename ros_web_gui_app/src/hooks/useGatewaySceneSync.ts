import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import type { Snapshot } from '../types/GatewayTypes';
import { getSafeMaxTextureSize, downsampleOccupancyGrid } from '../utils/mapDownsample';
import { mapTextureCache } from '../utils/mapTextureCache';

interface GatewaySceneSyncOptions {
  scene: THREE.Scene | null;
  snapshot: Snapshot | null;
  navGoal: { x: number; y: number; yaw: number } | null;
  /** 当 rosbridge 直连已接管 voxel_grid 时设为 true，网关不再渲染 */
  skipVoxelGrid?: boolean;
  /** 当 OccupancyGridLayer 接管地图渲染时为 true（避免双图重叠+坐标系冲突） */
  skipMap?: boolean;
}

/**
 * Load a grayscale PNG map image, downsample if needed, convert to THREE.DataTexture.
 * Cached by URL base — entering Map Editor won't re-decode the same PNG.
 */
function loadPngMapTexture(
  url: string,
  origWidth: number,
  origHeight: number,
  maxTexSize: number,
): Promise<{ texture: THREE.DataTexture; width: number; height: number }> {
  // Invalidate old cache entry for this base URL when map changes
  const baseUrl = url.split('?')[0]!
  for (const [key] of mapTextureCache) {
    if (key.startsWith(baseUrl)) mapTextureCache.delete(key)
  }
  const cacheKey = url
  const cached = mapTextureCache.get(cacheKey)
  if (cached) return Promise.resolve(cached)
  return new Promise((resolve, reject) => {
    // Calculate downsampling before loading
    const factor = Math.max(1, Math.ceil(Math.max(origWidth, origHeight) / maxTexSize));
    const dw = Math.floor(origWidth / factor);
    const dh = Math.floor(origHeight / factor);

    const img = new Image();
    img.crossOrigin = 'anonymous';
    img.onload = () => {
      // Draw full image to a downsampled canvas — browser does the scaling in GPU
      const canvas = document.createElement('canvas');
      canvas.width = dw;
      canvas.height = dh;
      const ctx = canvas.getContext('2d');
      if (!ctx) { reject(new Error('No 2d context')); return; }
      // disable image smoothing for nearest-neighbor downsampling (preserves sharp edges)
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, 0, 0, origWidth, origHeight, 0, 0, dw, dh);
      const imageData = ctx.getImageData(0, 0, dw, dh);
      // Flip Y: canvas row 0 (top) → RGBA row (dh-1) (bottom), matching OccupancyGrid convention
      const rgba = new Uint8Array(dw * dh * 4);
      for (let cy = 0; cy < dh; cy++) {
        const srcRow = cy * dw * 4;
        const dstRow = (dh - 1 - cy) * dw * 4;
        for (let cx = 0; cx < dw; cx++) {
          const gray = imageData.data[srcRow + cx * 4]!;
          const off = dstRow + cx * 4;
          if (gray <= 1) {
            rgba[off] = 20; rgba[off + 1] = 20; rgba[off + 2] = 20; rgba[off + 3] = 255;
          } else if (gray >= 250) {
            rgba[off] = 248; rgba[off + 1] = 248; rgba[off + 2] = 248; rgba[off + 3] = 255;
          } else {
            rgba[off] = 128; rgba[off + 1] = 128; rgba[off + 2] = 128; rgba[off + 3] = 80;
          }
        }
      }
      const texture = new THREE.DataTexture(rgba, dw, dh, THREE.RGBAFormat);
      texture.flipY = false;  // data is already bottom-first (matching OccupancyGrid)
      texture.needsUpdate = true;
      texture.minFilter = THREE.LinearFilter;
      texture.magFilter = THREE.LinearFilter;
      const result = { texture, width: dw, height: dh };
      mapTextureCache.set(cacheKey, result);
      resolve(result);
    };
    img.onerror = () => reject(new Error('Failed to load map PNG: ' + url));
    img.src = url;
  });
}

/**
 * 将 Gateway 遥测数据直接渲染到 Three.js 场景中。
 * 网关模式下的地图始终由此组件渲染，不受 rosbridge 直连状态影响。
 */
export function useGatewaySceneSync({ scene, snapshot, navGoal, skipVoxelGrid, skipMap }: GatewaySceneSyncOptions) {
  const mapMeshRef = useRef<THREE.Mesh | null>(null);
  const mapTextureRef = useRef<THREE.DataTexture | null>(null);
  const robotMarkerRef = useRef<THREE.Group | null>(null);
  const lidarPointsRef = useRef<THREE.Points | null>(null);
  const globalPlanLineRef = useRef<THREE.Line | null>(null);
  const localPlanLineRef = useRef<THREE.Line | null>(null);
  const voxelGridPointsRef = useRef<THREE.Points | null>(null);
  const navPointMarkersRef = useRef<THREE.Group | null>(null);
  const navGoalMarkerRef = useRef<THREE.Group | null>(null);
  const lastMapDataRef = useRef<string>('');
  // 数据比对 refs：避免无变化时重建 THREE.js 对象
  const lastGlobalPlanRef = useRef<string>('');
  const lastLocalPlanRef = useRef<string>('');
  const lastVoxelGridRef = useRef<string>('');
  const lastNavPointsRef = useRef<string>('');
  const lastNavModeRef = useRef<string>('');
  const lastNavGoalRef = useRef<string>('');

  // 渲染栅格地图（网关遥测数据）
  useEffect(() => {
    if (skipMap) return;
    const map = snapshot?.runtime?.map;

    if (!map || !scene) return;

    const hasDirectData = map.data && map.data.length > 0;
    const hasMapUrl = !!(map as any).mapUrl;

    if (!hasDirectData && !hasMapUrl) return;

    // 用地图更新时间戳检测变化，无变化时跳过重绘
    const mapUpdatedAt = snapshot?.runtime?.mapUpdatedAt;
    if (mapUpdatedAt && mapUpdatedAt === lastMapDataRef.current) return;
    lastMapDataRef.current = mapUpdatedAt ?? '';

    let { width, height, resolution, origin } = map;
    let gridData: number[] | Int8Array | null = hasDirectData ? map.data : null;

    // Downsample if map exceeds mobile GPU max texture size
    const maxTexSize = getSafeMaxTextureSize();
    let scaleCompensation = 1;
    if (gridData && (width > maxTexSize || height > maxTexSize)) {
      const result = downsampleOccupancyGrid(width, height, gridData, maxTexSize);
      scaleCompensation = width / result.width;
      console.log(
        `[GatewaySceneSync] Downsampled map from ${width}x${height} to ${result.width}x${result.height} ` +
        `(max texture size: ${maxTexSize})`
      );
      width = result.width;
      height = result.height;
      gridData = result.data;
      resolution = resolution * scaleCompensation;
    }

    const mapUrl = (map as any).mapUrl as string | undefined;
    const effectiveRes = resolution;

    const renderWithTexture = (texture: THREE.DataTexture, rw?: number, rh?: number, rres?: number) => {
      const rWidth = rw ?? width;
      const rHeight = rh ?? height;
      const rRes = rres ?? effectiveRes;
      // 移除旧 mesh
      if (mapMeshRef.current) {
        scene.remove(mapMeshRef.current);
        mapMeshRef.current.geometry?.dispose();
        if (Array.isArray(mapMeshRef.current.material)) {
          mapMeshRef.current.material.forEach((m) => m.dispose());
        } else {
          mapMeshRef.current.material?.dispose();
        }
      }
      if (mapTextureRef.current) {
        mapTextureRef.current.dispose();
      }
      mapTextureRef.current = texture;

      const mapWidth = rWidth * rRes;
      const mapHeight = rHeight * rRes;
      const geometry = new THREE.PlaneGeometry(mapWidth, mapHeight);
      const material = new THREE.MeshBasicMaterial({
        map: texture,
        transparent: true,
        depthTest: false,
        depthWrite: false,
        side: THREE.DoubleSide,
      });

      const mesh = new THREE.Mesh(geometry, material);
      mesh.renderOrder = 1;
      mesh.position.set(
        origin.x + mapWidth / 2,
        origin.y + mapHeight / 2,
        0.001
      );
      mesh.name = 'gateway_map';
      scene.add(mesh);
      mapMeshRef.current = mesh;

      console.log('[GatewaySceneSync] map rendered', { width: rWidth, height: rHeight, res: rRes, origin, via: gridData ? 'data' : 'png_url' });
    };

    // If we have raw data (small map), render directly
    if (gridData) {
      // 创建纹理像素数据
      const pixelData = new Uint8Array(width * height * 4);
      for (let i = 0; i < gridData.length; i++) {
        const v = gridData[i]!;
        const offset = i * 4;
        if (v < 0) {
          pixelData[offset] = 128;
          pixelData[offset + 1] = 128;
          pixelData[offset + 2] = 128;
          pixelData[offset + 3] = 80;
        } else if (v > 65) {
          const c = Math.round(255 * (1 - (v - 65) / 35));
          pixelData[offset] = c;
          pixelData[offset + 1] = c;
          pixelData[offset + 2] = c;
          pixelData[offset + 3] = 255;
        } else if (v >= 0 && v <= 45) {
          pixelData[offset] = 248;
          pixelData[offset + 1] = 248;
          pixelData[offset + 2] = 248;
          pixelData[offset + 3] = 255;
        } else {
          const c = Math.round(240 - ((v - 45) / 20) * 100);
          pixelData[offset] = c;
          pixelData[offset + 1] = c;
          pixelData[offset + 2] = c;
          pixelData[offset + 3] = 255;
        }
      }

      const texture = new THREE.DataTexture(pixelData, width, height, THREE.RGBAFormat);
      texture.flipY = false;
      texture.needsUpdate = true;
      texture.minFilter = THREE.LinearFilter;
      texture.magFilter = THREE.LinearFilter;
      renderWithTexture(texture);
    } else if (mapUrl) {
      // Large map — load PNG as image with built-in downsampling
      const maxTexSize = getSafeMaxTextureSize();
      loadPngMapTexture(mapUrl, width, height, maxTexSize)
        .then(({ texture, width: newW, height: newH }) => {
          // Adjust resolution to account for downsampling
          const factor = width / newW;
          renderWithTexture(texture, newW, newH, resolution * factor);
        })
        .catch((err) => console.error('[GatewaySceneSync] map PNG load failed:', err));
    }
  }, [scene, snapshot?.runtime?.map]);

  // 渲染机器人位置标记（大圆 + 方向箭头）
  useEffect(() => {
    const pose = snapshot?.runtime?.tfPose;
    if (!pose || !scene) return;

    if (!robotMarkerRef.current) {
      const group = new THREE.Group();
      group.name = 'gateway_robot';
      const z = 0;
      const ro = 9999;

      // 机器人底盘（空心大圆）
      const ringGeom = new THREE.RingGeometry(0.25, 0.30, 32);
      const ringMat = new THREE.MeshBasicMaterial({ color: 0x00ff00, side: THREE.DoubleSide, depthTest: false, transparent: true });
      const ring = new THREE.Mesh(ringGeom, ringMat);
      ring.renderOrder = ro;
      ring.position.z = z;
      group.add(ring);

      // 方向箭头（长三角形）
      const arrowShape = new THREE.Shape();
      arrowShape.moveTo(0.55, 0);
      arrowShape.lineTo(-0.20, -0.20);
      arrowShape.lineTo(-0.05, 0);
      arrowShape.lineTo(-0.20, 0.20);
      arrowShape.closePath();
      const arrowGeom = new THREE.ShapeGeometry(arrowShape);
      const arrowMat = new THREE.MeshBasicMaterial({ color: 0xff4444, side: THREE.DoubleSide, depthTest: false, transparent: true });
      const arrow = new THREE.Mesh(arrowGeom, arrowMat);
      arrow.renderOrder = ro + 1;
      arrow.position.z = z + 0.001;
      group.add(arrow);

      // 中心小圆点
      const dotGeom = new THREE.CircleGeometry(0.06, 16);
      const dotMat = new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide, depthTest: false, transparent: true });
      const dot = new THREE.Mesh(dotGeom, dotMat);
      dot.renderOrder = ro + 2;
      dot.position.z = z + 0.002;
      group.add(dot);

      scene.add(group);
      robotMarkerRef.current = group;
    }

    robotMarkerRef.current.position.set(pose.x, pose.y, 0.1);
    robotMarkerRef.current.rotation.set(0, 0, pose.yaw);
    robotMarkerRef.current.renderOrder = 9999;
  }, [scene, snapshot?.runtime?.tfPose]);

  // 激光雷达点云（已禁用 - 不显示蓝色点云）
  // useEffect(() => {
  //   const lidar = snapshot?.runtime?.lidar;
  //   if (!lidar || lidar.length === 0 || !scene) return;

  //   if (lidarPointsRef.current) {
  //     scene.remove(lidarPointsRef.current);
  //     lidarPointsRef.current.geometry?.dispose();
  //     (lidarPointsRef.current.material as THREE.Material)?.dispose();
  //   }

  //   const positions = new Float32Array(lidar.length * 3);
  //   for (let i = 0; i < lidar.length; i++) {
  //     positions[i * 3] = lidar[i]!.x;
  //     positions[i * 3 + 1] = lidar[i]!.y;
  //     positions[i * 3 + 2] = 0.002;
  //   }

  //   const geom = new THREE.BufferGeometry();
  //   geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  //   const mat = new THREE.PointsMaterial({ color: 0x00ffff, size: 0.05, depthTest: false, depthWrite: false, transparent: true, opacity: 0.9 });
  //   const points = new THREE.Points(geom, mat);
  //   points.renderOrder = 9998;
  //   points.position.z = 0.2;
  //   points.name = 'gateway_lidar';
  //   scene.add(points);
  //   lidarPointsRef.current = points;
  // }, [scene, snapshot?.runtime?.lidar]);

  // 渲染全局路径（蓝色）
  useEffect(() => {
    const plan = snapshot?.runtime?.globalPlan;
    if (!scene) return;
    if (!plan || plan.length < 2) {
      if (globalPlanLineRef.current) { scene.remove(globalPlanLineRef.current); globalPlanLineRef.current.geometry?.dispose(); globalPlanLineRef.current = null; }
      lastGlobalPlanRef.current = '';
      return;
    }
    const key = JSON.stringify(plan);
    if (key === lastGlobalPlanRef.current) return;
    lastGlobalPlanRef.current = key;
    const pts = plan.map((p) => new THREE.Vector3(p.x, p.y, 0.004));
    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    if (globalPlanLineRef.current) {
      scene.remove(globalPlanLineRef.current);
      globalPlanLineRef.current.geometry?.dispose();
      (globalPlanLineRef.current.material as THREE.Material)?.dispose();
    }
    const mat = new THREE.LineBasicMaterial({ color: 0x0066ff, linewidth: 1, depthTest: false, transparent: true, opacity: 0.9 });
    const line = new THREE.Line(geom, mat);
    line.renderOrder = 100;
    line.name = 'gateway_global_plan';
    scene.add(line);
    globalPlanLineRef.current = line;
  }, [scene, snapshot?.runtime?.globalPlan]);

  // 渲染局部路径（绿色）
  useEffect(() => {
    const plan = snapshot?.runtime?.localPlan;
    if (!scene) return;
    if (!plan || plan.length < 2) {
      if (localPlanLineRef.current) { scene.remove(localPlanLineRef.current); localPlanLineRef.current.geometry?.dispose(); localPlanLineRef.current = null; }
      lastLocalPlanRef.current = '';
      return;
    }
    const key = JSON.stringify(plan);
    if (key === lastLocalPlanRef.current) return;
    lastLocalPlanRef.current = key;
    const pts = plan.map((p) => new THREE.Vector3(p.x, p.y, 0.005));
    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    if (localPlanLineRef.current) {
      scene.remove(localPlanLineRef.current);
      localPlanLineRef.current.geometry?.dispose();
      (localPlanLineRef.current.material as THREE.Material)?.dispose();
    }
    const mat = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 1, depthTest: false, transparent: true, opacity: 0.9 });
    const line = new THREE.Line(geom, mat);
    line.renderOrder = 100;
    line.name = 'gateway_local_plan';
    scene.add(line);
    localPlanLineRef.current = line;
  }, [scene, snapshot?.runtime?.localPlan]);

  // 渲染体素点云（橙色）
  // 当 rosbridge 直连可用时跳过，由 PointCloud2Layer 以更高点数上限接管
  useEffect(() => {
    if (skipVoxelGrid) return;
    const voxel = snapshot?.runtime?.voxelGrid;
    if (!scene) return;
    if (!voxel || voxel.length === 0) {
      if (voxelGridPointsRef.current) { scene.remove(voxelGridPointsRef.current); voxelGridPointsRef.current.geometry?.dispose(); (voxelGridPointsRef.current.material as THREE.Material)?.dispose(); voxelGridPointsRef.current = null; }
      lastVoxelGridRef.current = '';
      return;
    }
    const key = JSON.stringify(voxel);
    if (key === lastVoxelGridRef.current) return;
    lastVoxelGridRef.current = key;
    const positions = new Float32Array(voxel.length * 3);
    for (let i = 0; i < voxel.length; i++) {
      positions[i * 3] = voxel[i]!.x;
      positions[i * 3 + 1] = voxel[i]!.y;
      positions[i * 3 + 2] = 0.006;
    }
    if (voxelGridPointsRef.current) {
      scene.remove(voxelGridPointsRef.current);
      voxelGridPointsRef.current.geometry?.dispose();
      (voxelGridPointsRef.current.material as THREE.Material)?.dispose();
    }
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const mat = new THREE.PointsMaterial({ color: 0xff6600, size: 0.08, depthTest: false, depthWrite: false, transparent: true, opacity: 0.9 });
    const points = new THREE.Points(geom, mat);
    points.renderOrder = 10000;
    points.position.z = 0.3;
    points.name = 'gateway_voxel_grid';
    scene.add(points);
    voxelGridPointsRef.current = points;
  }, [scene, snapshot?.runtime?.voxelGrid, skipVoxelGrid]);

  // 渲染导航点
  useEffect(() => {
    const points = snapshot?.navPoints;
    const navMode = snapshot?.runtime?.navMode;
    if (!scene) return;
    // 只在多点导航时显示
    if (navMode !== 'multi') {
      if (navPointMarkersRef.current) { scene.remove(navPointMarkersRef.current); navPointMarkersRef.current.traverse((c) => { if (c instanceof THREE.Mesh) { c.geometry?.dispose(); (c.material as THREE.Material)?.dispose(); } }); navPointMarkersRef.current = null; }
      lastNavPointsRef.current = '';
      lastNavModeRef.current = '';
      return;
    }
    const key = JSON.stringify(points) + '|' + navMode;
    if (key === lastNavPointsRef.current && navMode === lastNavModeRef.current) return;
    lastNavPointsRef.current = key;
    lastNavModeRef.current = navMode;
    // 清除旧标记
    if (navPointMarkersRef.current) {
      scene.remove(navPointMarkersRef.current);
      navPointMarkersRef.current.traverse((c) => {
        if (c instanceof THREE.Mesh) { c.geometry?.dispose(); (c.material as THREE.Material)?.dispose(); }
      });
      navPointMarkersRef.current = null;
    }
    if (!points || points.length === 0) return;

    const group = new THREE.Group();
    group.name = 'gateway_nav_points';
    const z = 0.5;
    const baseRenderOrder = 500;
    points.forEach((p) => {
      const ringGeom = new THREE.RingGeometry(0.18, 0.22, 32);
      const ringMat = new THREE.MeshBasicMaterial({ color: 0xffff00, depthTest: false, transparent: true, opacity: 0.9 });
      const ring = new THREE.Mesh(ringGeom, ringMat);
      ring.position.set(p.x, p.y, z);
      ring.renderOrder = baseRenderOrder;
      group.add(ring);
      const dotGeom = new THREE.CircleGeometry(0.08, 16);
      const dotMat = new THREE.MeshBasicMaterial({ color: 0xff4400, depthTest: false, transparent: true, opacity: 0.9 });
      const dot = new THREE.Mesh(dotGeom, dotMat);
      dot.position.set(p.x, p.y, z + 0.001);
      dot.renderOrder = baseRenderOrder + 1;
      group.add(dot);
      if (p.yaw !== undefined && p.yaw !== null) {
        const arrShape = new THREE.Shape();
        arrShape.moveTo(0.3, 0);
        arrShape.lineTo(-0.08, -0.06);
        arrShape.lineTo(-0.02, 0);
        arrShape.lineTo(-0.08, 0.06);
        arrShape.closePath();
        const arrGeom = new THREE.ShapeGeometry(arrShape);
        const arrMat = new THREE.MeshBasicMaterial({ color: 0xff0000, depthTest: false, transparent: true, opacity: 1.0 });
        const arrow = new THREE.Mesh(arrGeom, arrMat);
        arrow.position.set(p.x, p.y, z + 0.002);
        arrow.rotation.set(0, 0, p.yaw);
        arrow.renderOrder = baseRenderOrder + 2;
        group.add(arrow);
      }
      // 文字标签（红色）
      if (p.name) {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        if (ctx) {
          const fontSize = 54;
          ctx.font = `bold ${fontSize}px Arial`;
          const textWidth = ctx.measureText(p.name).width;
          canvas.width = Math.max(64, Math.ceil(textWidth + 20));
          canvas.height = Math.ceil(fontSize * 1.6);
          // canvas 尺寸改变会重置 context，重新设置
          ctx.font = `bold ${fontSize}px Arial`;
          ctx.textAlign = 'center';
          ctx.textBaseline = 'middle';
          // 暗色投影
          ctx.lineWidth = 3;
          ctx.strokeStyle = 'rgba(0, 0, 0, 0.6)';
          ctx.strokeText(p.name, canvas.width / 2, canvas.height / 2);
          // 红色填充
          ctx.fillStyle = '#ff3333';
          ctx.fillText(p.name, canvas.width / 2, canvas.height / 2);
        }
        const texture = new THREE.CanvasTexture(canvas);
        texture.minFilter = THREE.LinearFilter;
        const spriteMat = new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true, opacity: 0.95 });
        const sprite = new THREE.Sprite(spriteMat);
        const aspect = canvas.width / canvas.height;
        sprite.position.set(p.x, p.y + 0.32, z + 0.01);
        sprite.scale.set(aspect * 0.22, 0.22, 1);
        sprite.renderOrder = baseRenderOrder + 10;
        group.add(sprite);
      }
    });
    scene.add(group);
    navPointMarkersRef.current = group;
  }, [scene, snapshot?.navPoints, snapshot?.runtime?.navMode]);

  // 渲染 2D Nav Goal 标记（大红菱形 + 箭头）
  // 多点导航模式下不显示，避免与导航点标记混淆
  useEffect(() => {
    if (!scene) return;
    const navMode = snapshot?.runtime?.navMode;
    // 多点导航模式或无目标时不渲染
    if (navMode === 'multi' || !navGoal) {
      if (navGoalMarkerRef.current) { scene.remove(navGoalMarkerRef.current); navGoalMarkerRef.current.traverse((c) => { if (c instanceof THREE.Mesh) { c.geometry?.dispose(); (c.material as THREE.Material)?.dispose(); } }); navGoalMarkerRef.current = null; }
      lastNavGoalRef.current = '';
      return;
    }
    const key = JSON.stringify(navGoal) + '|' + navMode;
    if (key === lastNavGoalRef.current) return;
    lastNavGoalRef.current = key;
    // 始终先清理旧标记
    if (navGoalMarkerRef.current) {
      scene.remove(navGoalMarkerRef.current);
      navGoalMarkerRef.current.traverse((c) => { if (c instanceof THREE.Mesh) { c.geometry?.dispose(); (c.material as THREE.Material)?.dispose(); } });
      navGoalMarkerRef.current = null;
    }
    const group = new THREE.Group();
    const z = 0.6;
    const circle = new THREE.Mesh(
      new THREE.RingGeometry(0.15, 0.22, 32),
      new THREE.MeshBasicMaterial({ color: 0xff0000, depthTest: false, transparent: true, opacity: 0.9, side: THREE.DoubleSide }),
    );
    circle.renderOrder = 600; circle.position.z = z; group.add(circle);
    const arrShape = new THREE.Shape();
    arrShape.moveTo(0.5, 0); arrShape.lineTo(-0.1, -0.1); arrShape.lineTo(-0.03, 0); arrShape.lineTo(-0.1, 0.1); arrShape.closePath();
    const arrow = new THREE.Mesh(new THREE.ShapeGeometry(arrShape), new THREE.MeshBasicMaterial({ color: 0xff0000, depthTest: false, transparent: true, opacity: 0.9, side: THREE.DoubleSide }));
    arrow.renderOrder = 601; arrow.position.z = z + 0.001; arrow.rotation.set(0, 0, navGoal.yaw); group.add(arrow);
    group.position.set(navGoal.x, navGoal.y, 0);
    scene.add(group);
    navGoalMarkerRef.current = group;
  }, [scene, navGoal, snapshot?.runtime?.navMode]);

  useEffect(() => {
    return () => {
      if (mapMeshRef.current && scene) {
        scene.remove(mapMeshRef.current);
        mapMeshRef.current.geometry?.dispose();
        if (Array.isArray(mapMeshRef.current.material)) {
          mapMeshRef.current.material.forEach((m) => m.dispose());
        } else {
          mapMeshRef.current.material?.dispose();
        }
      }
      if (mapTextureRef.current) mapTextureRef.current.dispose();
      if (robotMarkerRef.current && scene) {
        scene.remove(robotMarkerRef.current);
        robotMarkerRef.current.traverse((child) => {
          if (child instanceof THREE.Mesh) {
            child.geometry?.dispose();
            if (Array.isArray(child.material)) {
              child.material.forEach((m) => m.dispose());
            } else {
              (child.material as THREE.Material)?.dispose();
            }
          }
        });
      }
      if (lidarPointsRef.current && scene) {
        scene.remove(lidarPointsRef.current);
        lidarPointsRef.current.geometry?.dispose();
        (lidarPointsRef.current.material as THREE.Material)?.dispose();
      }
      if (globalPlanLineRef.current && scene) {
        scene.remove(globalPlanLineRef.current);
        globalPlanLineRef.current.geometry?.dispose();
        (globalPlanLineRef.current.material as THREE.Material)?.dispose();
      }
      if (localPlanLineRef.current && scene) {
        scene.remove(localPlanLineRef.current);
        localPlanLineRef.current.geometry?.dispose();
        (localPlanLineRef.current.material as THREE.Material)?.dispose();
      }
      if (voxelGridPointsRef.current && scene) {
        scene.remove(voxelGridPointsRef.current);
        voxelGridPointsRef.current.geometry?.dispose();
        (voxelGridPointsRef.current.material as THREE.Material)?.dispose();
      }
      if (navPointMarkersRef.current && scene) {
        scene.remove(navPointMarkersRef.current);
        navPointMarkersRef.current.traverse((c) => {
          if (c instanceof THREE.Mesh) { c.geometry?.dispose(); (c.material as THREE.Material)?.dispose(); }
        });
      }
      if (navGoalMarkerRef.current && scene) {
        scene.remove(navGoalMarkerRef.current);
        navGoalMarkerRef.current.traverse((c) => {
          if (c instanceof THREE.Mesh) { c.geometry?.dispose(); (c.material as THREE.Material)?.dispose(); }
        });
      }
    };
  }, [scene]);
}
