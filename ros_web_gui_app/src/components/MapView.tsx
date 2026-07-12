import { useCallback, useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { toast } from 'react-toastify';
import { RosbridgeConnection } from '../utils/RosbridgeConnection';
import { TF2JS } from '../utils/tf2js';
import { LayerManager } from './layers/LayerManager';
import type { LayerConfigMap } from '../types/LayerConfig';
import { MapEditor } from './MapEditor';
import { ImageDisplay } from './ImageDisplay';
import { TopoPointInfoPanel } from './TopoPointInfoPanel';
import { NavParamsPanel } from './NavParamsPanel';
import { DEFAULT_LAYER_CONFIGS } from '../constants/layerConfigs';
import { loadLayerConfigs, saveLayerConfigs, saveImagePositions, type ImagePositionsMap } from '../utils/layerConfigStorage';
import { useLayerConfigSync } from '../hooks/useLayerConfigSync';
import { useInitialization } from '../hooks/useInitialization';
import { useImageLayers } from '../hooks/useImageLayers';

import { useViewMode } from '../hooks/useViewMode';
import { useConnectionInit } from '../hooks/useConnectionInit';
import { useGatewaySceneSync } from '../hooks/useGatewaySceneSync';
import { useGatewayContext } from './gateway/GatewayProvider';
import { MapManager, type OccupancyGrid } from '../utils/MapManager';
import { MappingControl } from './gateway/MappingControl/MappingControl';
import { MapManagerPanel } from './gateway/MapManager/MapManagerPanel';
import { NavControl } from './gateway/NavControl/NavControl';
import { LiDARControl } from './gateway/LiDARControl/LiDARControl';
import { UserManager } from './gateway/UserManager/UserManager';
import { MotionControl } from './gateway/MotionControl/MotionControl';
import './MapView.css';

type GatewayPanel = 'mapping' | 'map-manager' | 'nav-control' | 'lidar-control' | 'motion-control' | 'users';

const gatewayButtons: { id: GatewayPanel; label: string; icon: string }[] = [
  { id: 'mapping', label: '建图', icon: '🗺️' },
  { id: 'lidar-control', label: '雷达', icon: '📡' },
  { id: 'motion-control', label: '运控', icon: '🎮' },
  { id: 'nav-control', label: '导航', icon: '🧭' },
  { id: 'map-manager', label: '地图', icon: '📁' },
  { id: 'users', label: '用户', icon: '👤' },
];

function GatewayPanelRenderer({ panel }: { panel: GatewayPanel }) {
  switch (panel) {
    case 'mapping': return <MappingControl />;
    case 'nav-control': return <NavControl />;
    case 'lidar-control': return <LiDARControl />;
    case 'motion-control': return <MotionControl />;
    case 'map-manager': return <MapManagerPanel />;
    case 'users': return <UserManager />;
    default: return null;
  }
}

interface MapViewProps {
  connection: RosbridgeConnection | null;
  gatewayToken: string | null;
  onGatewayLogin: (token: string) => void;
  onGatewayLogout: () => void;
}

export function MapView({ connection, gatewayToken, onGatewayLogin, onGatewayLogout }: MapViewProps) {
  const [gatewayPanel, setGatewayPanel] = useState<GatewayPanel | null>(null);
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  const [showGatewayLogin, setShowGatewayLogin] = useState(false);
  const [gwUsername, setGwUsername] = useState('admin');
  const [gwPassword, setGwPassword] = useState('admin123');
  const [gwLoginError, setGwLoginError] = useState('');

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const gwPanelRef = useRef<HTMLDivElement>(null);
  const layerManagerRef = useRef<LayerManager | null>(null);
  const [layerConfigs, setLayerConfigs] = useState<LayerConfigMap>(() => {
    const saved = loadLayerConfigs();
    if (saved) {
      const merged: LayerConfigMap = { ...DEFAULT_LAYER_CONFIGS };
      for (const [key, defaultConfig] of Object.entries(DEFAULT_LAYER_CONFIGS)) {
        if (saved[key]) merged[key] = { ...defaultConfig, ...saved[key] };
      }
      for (const [key, config] of Object.entries(saved)) {
        if (!DEFAULT_LAYER_CONFIGS[key] && (config.id === 'image' || config.id === 'cmd_vel')) {
          merged[key] = config;
        }
      }
      // 强制 map_cloud 使用累加模式
      if (merged.map_cloud) {
        merged.map_cloud.accumulate = true;
        merged.map_cloud.targetFrame = 'camera_init';
      }
      // 强制 slam_path 使用 camera_init 坐标系
      if (merged.slam_path) {
        merged.slam_path.mapFrame = 'camera_init';
      }
      // 强制 voxel_grid 使用 map 坐标系 + 启用
      if (merged.voxel_grid) {
        merged.voxel_grid.enabled = true;
        merged.voxel_grid.targetFrame = 'map';
      }
      // 强制 robot 启用 + 匹配实际 TF 树 camera_init→body
      if (merged.robot) {
        merged.robot.enabled = true;
        merged.robot.mapFrame = 'camera_init';
        merged.robot.baseFrame = 'body';
      }
      return merged;
    }
    console.log('[MapView] using default configs, map_cloud:', JSON.stringify(DEFAULT_LAYER_CONFIGS.map_cloud));
    return DEFAULT_LAYER_CONFIGS;
  });
  const layerConfigsRef = useRef<LayerConfigMap>(layerConfigs);
  const [viewMode, setViewMode] = useState<'2d' | '3d'>('2d');
  const viewModeRef = useRef<'2d' | '3d'>('2d');
  const [showEditor, setShowEditor] = useState(false);
  const [showNavParams, setShowNavParams] = useState(false);
  const [focusRobot, setFocusRobot] = useState(false);
  const [pcDensity, setPcDensity] = useState<number>(() => {
    const saved = loadLayerConfigs();
    return (saved?.map_cloud?.maxTotalPoints as number) ?? (DEFAULT_LAYER_CONFIGS.map_cloud?.maxTotalPoints as number) ?? 5000000;
  });
  const [showDensitySlider, setShowDensitySlider] = useState(false);
  const [scanDistance, setScanDistance] = useState<number>(5);
  const [showScanDistSlider, setShowScanDistSlider] = useState(false);
  const [navTolerance, setNavTolerance] = useState<number>(0.2);
  const [showNavTolSlider, setShowNavTolSlider] = useState(false);
  const [obstacleDist, setObstacleDist] = useState<number>(0.15);
  const [showObstacleDistSlider, setShowObstacleDistSlider] = useState(false);
  const [mouseWorldPos, setMouseWorldPos] = useState<{ x: number; y: number } | null>(null);
  const [robotPos, setRobotPos] = useState<{ x: number; y: number; theta: number } | null>(null);
  const focusRobotRef = useRef(false);
  const [selectedTopoPoint, setSelectedTopoPoint] = useState<{
    name: string;
    x: number;
    y: number;
    theta: number;
  } | null>(null);
  const [selectedTopoRoute, setSelectedTopoRoute] = useState<{
    from_point: string;
    to_point: string;
    route_info: {
      controller: string;
      goal_checker: string;
      speed_limit: number;
    };
  } | null>(null);
  const raycasterRef = useRef<THREE.Raycaster | null>(null);
  const imagePositionsRef = useRef<Map<string, { x: number; y: number; scale: number }>>(new Map());
  const timeoutRefsRef = useRef<Set<ReturnType<typeof setTimeout>>>(new Set());
  const [relocalizeMode, setRelocalizeMode] = useState(false);
  const [navGoalMode, setNavGoalMode] = useState(false);
  const [navGoal, setNavGoal] = useState<{ x: number; y: number; yaw: number } | null>(null);
  const [sysMem, setSysMem] = useState<{ usedPercent: number; freeGB: string; totalGB: string; heapMB: number; load1m: number }>({ usedPercent: 0, freeGB: '0', totalGB: '0', heapMB: 0, load1m: 0 });
  const relocalizeModeRef = useRef(false);
  const navGoalModeRef = useRef(false);
  const poseEstimateRef = useRef<{ x: number; y: number; yaw: number } | null>(null);
  const isEstimatingRef = useRef(false);
  const estimateStartRef = useRef<THREE.Vector3 | null>(null);
  const estimateArrowRef = useRef<THREE.Line | null>(null);
  const estimateLineObjRef = useRef<THREE.Line | null>(null);
  const estimateModeRef = useRef<'relocalize' | 'navgoal' | null>(null);
  const initialposeTopicRef = useRef<string>('/initialpose');
  const gatewayTokenRef = useRef<string | null>(gatewayToken);
  gatewayTokenRef.current = gatewayToken;

  useInitialization(initialposeTopicRef, imagePositionsRef);

  const imageLayers = useImageLayers(layerConfigs, imagePositionsRef);

  useViewMode(viewMode, viewModeRef, controlsRef, cameraRef);

  // 左下角坐标显示：鼠标移动/触摸时转换屏幕坐标→世界坐标
  const screenToWorld = useCallback((clientX: number, clientY: number) => {
    const canvas = canvasRef.current;
    const camera = cameraRef.current;
    const raycaster = raycasterRef.current;
    if (!canvas || !camera || !raycaster) return null;
    const rect = canvas.getBoundingClientRect();
    const m = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(m, camera);
    const pt = new THREE.Vector3();
    raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), pt);
    return pt;
  }, []);

  const handleCanvasMouseMove = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const pt = screenToWorld(e.clientX, e.clientY);
    if (pt) setMouseWorldPos({ x: pt.x, y: pt.y });
  }, [screenToWorld]);

  const handleCanvasTouch = useCallback((e: React.TouchEvent<HTMLCanvasElement>) => {
    const t = e.touches[0] || e.changedTouches[0];
    if (!t) return;
    const pt = screenToWorld(t.clientX, t.clientY);
    if (pt) setMouseWorldPos({ x: pt.x, y: pt.y });
  }, [screenToWorld]);

  useEffect(() => {
    if (!canvasRef.current) return;

    const canvas = canvasRef.current;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x808080);
    sceneRef.current = scene;

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 5, 10);
    directionalLight.castShadow = false;
    scene.add(directionalLight);

    const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight2.position.set(-5, -5, 5);
    directionalLight2.castShadow = false;
    scene.add(directionalLight2);

    THREE.Object3D.DEFAULT_UP = new THREE.Vector3(0, 0, 1);

    const camera = new THREE.PerspectiveCamera(75, canvas.clientWidth / canvas.clientHeight, 0.1, 1000);
    camera.position.set(0, 0, 10);
    camera.up.set(0, 0, 1);
    camera.lookAt(0, 0, 0);
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    renderer.setClearColor(0x808080, 1);
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    // Cap pixel ratio on mobile to reduce GPU memory pressure.
    // High-DPI phones (3x) render at 9× the pixels of 1x, which
    // causes WebGL texture size limits and OOM on large maps.
    const isMobile = window.innerWidth < 1024;
    const pixelRatio = isMobile
      ? Math.min(window.devicePixelRatio, 2)
      : window.devicePixelRatio;
    renderer.setPixelRatio(pixelRatio);
    rendererRef.current = renderer;

    const controls = new OrbitControls(camera, canvas);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    // minDistance 控制最大放大比例（值越小，放大倍数越大）
    // maxDistance 控制最大缩小比例（值越大，缩小倍数越大）
    controls.minDistance = 0.1;
    controls.maxDistance = 1000;
    controls.target.set(0, 0, 0);
    controls.mouseButtons.LEFT = THREE.MOUSE.PAN;
    controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
    (controls as any).zoomToCursor = true;
    // 平板触摸手势：单指平移，双指旋转+捏合缩放
    (controls as any).touches = {
      ONE: THREE.TOUCH.PAN,
      TWO: THREE.TOUCH.DOLLY_ROTATE,
    };

    controls.update();

    controlsRef.current = controls;

    const raycaster = new THREE.Raycaster();
    raycasterRef.current = raycaster;

    const handleClick = (event: MouseEvent) => {
      if (!camera || !scene || !canvas) return;

      if (relocalizeMode) {
        return;
      }

      const rect = canvas.getBoundingClientRect();
      const mouse = new THREE.Vector2();
      mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

      raycaster.setFromCamera(mouse, camera);
      const intersects = raycaster.intersectObjects(scene.children, true);

      for (const intersect of intersects) {
        let obj = intersect.object;
        while (obj) {
          // 优先检测路线（因为路线在点下方）
          if (obj.userData.isTopoRoute && obj.userData.topoRoute) {
            const route = obj.userData.topoRoute;
            setSelectedTopoRoute({
              from_point: route.from_point,
              to_point: route.to_point,
              route_info: route.route_info,
            });
            setSelectedTopoPoint(null);

            // 更新 TopoLayer 的选中状态
            const topoLayer = layerManagerRef.current?.getLayer('topology');
            if (topoLayer && 'setSelectedRoute' in topoLayer) {
              (topoLayer as any).setSelectedRoute(route);
            }
            if (topoLayer && 'setSelectedPoint' in topoLayer) {
              (topoLayer as any).setSelectedPoint(null);
            }
            return;
          }
          if (obj.userData.isTopoPoint && obj.userData.topoPoint) {
            const point = obj.userData.topoPoint;
            setSelectedTopoPoint({
              name: point.name,
              x: point.x,
              y: point.y,
              theta: point.theta,
            });
            setSelectedTopoRoute(null);

            // 更新 TopoLayer 的选中状态
            const topoLayer = layerManagerRef.current?.getLayer('topology');
            if (topoLayer && 'setSelectedPoint' in topoLayer) {
              (topoLayer as any).setSelectedPoint(point);
            }
            if (topoLayer && 'setSelectedRoute' in topoLayer) {
              (topoLayer as any).setSelectedRoute(null);
            }
            return;
          }
          obj = obj.parent as THREE.Object3D;
        }
      }

      setSelectedTopoPoint(null);
      setSelectedTopoRoute(null);

      // 清除 TopoLayer 的选中状态
      const topoLayer = layerManagerRef.current?.getLayer('topology');
      if (topoLayer && 'setSelectedRoute' in topoLayer) {
        (topoLayer as any).setSelectedRoute(null);
      }
      if (topoLayer && 'setSelectedPoint' in topoLayer) {
        (topoLayer as any).setSelectedPoint(null);
      }
    };

    canvas.addEventListener('click', handleClick);

    // 重定位/导航目标：按下设位置+拖拽设方向+松开发布
    const el = estimateLineObjRef;
    const markerRef = { current: null as THREE.Mesh | null };
    const worldPos = (clientX: number, clientY: number) => {
      if (!camera || !canvas) return null;
      const rect = canvas.getBoundingClientRect();
      const m = new THREE.Vector2(
        ((clientX - rect.left) / rect.width) * 2 - 1,
        -((clientY - rect.top) / rect.height) * 2 + 1
      );
      raycaster.setFromCamera(m, camera);
      const pt = new THREE.Vector3();
      raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), pt);
      return pt;
    };
    const clearMarker = () => {
      [markerRef.current, el.current].forEach(o => {
        if (o && scene) { scene.remove(o); o.geometry?.dispose(); const mat = (o as THREE.Mesh).material; if (Array.isArray(mat)) mat.forEach((x: any) => x.dispose()); else (mat as any)?.dispose(); }
      });
      markerRef.current = null; el.current = null;
    };
    const updateArrow = (start: THREE.Vector3, end: THREE.Vector3) => {
      if (!scene) return;
      if (el.current) { scene.remove(el.current); el.current.geometry?.dispose(); (el.current.material as THREE.Material)?.dispose(); }
      const g = new THREE.BufferGeometry().setFromPoints([start.clone(), end.clone()]);
      el.current = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xff4444, depthTest: false, transparent: true }));
      el.current.renderOrder = 10000;
      scene.add(el.current);
    };
    const publishPose = async (pose: { x: number; y: number; yaw: number }, mode: string) => {
      if (mode === 'navgoal') {
        setNavGoal(pose);
        if (connection?.isConnected()) {
          const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, pose.yaw, 'XYZ'));
          connection.publish('/move_base_simple/goal', 'geometry_msgs/PoseStamped', {
            header: { stamp: { sec: Math.floor(Date.now()/1000), nsec: (Date.now()%1000)*1e6 }, frame_id: 'map' },
            pose: { position: { x: pose.x, y: pose.y, z: 0 }, orientation: { x: q.x, y: q.y, z: q.z, w: q.w } },
          });
          toast.success('导航目标已发布');
        } else if (gatewayTokenRef.current) {
          const { apiBase } = await import('../api/gatewayApi');
          const r = await fetch(`${apiBase}/api/nav/simple-goal`, { method: 'POST', headers: { 'Content-Type': 'application/json', Authorization: `Bearer ${gatewayTokenRef.current}` }, body: JSON.stringify({ x: pose.x, y: pose.y, yaw: pose.yaw, frame_id: 'map' }) });
          if (r.ok) toast.success('导航目标已发布'); else toast.error('发布失败');
        }
      } else {
        const msg = { header: { stamp: { sec: Math.floor(Date.now()/1000), nsec: (Date.now()%1000)*1e6 }, frame_id: 'map' }, pose: { pose: { position: { x: pose.x, y: pose.y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(pose.yaw/2), w: Math.cos(pose.yaw/2) } }, covariance: new Array(36).fill(0) } };
        if (connection?.isConnected()) { connection.publish('/initialpose', 'geometry_msgs/PoseWithCovarianceStamped', msg); toast.success('初始位姿已发布'); }
        else if (gatewayTokenRef.current) {
          const { apiBase } = await import('../api/gatewayApi');
          const r = await fetch(`${apiBase}/api/initialpose`, { method: 'POST', headers: { 'Content-Type': 'application/json', Authorization: `Bearer ${gatewayTokenRef.current}` }, body: JSON.stringify({ x: pose.x, y: pose.y, yaw: pose.yaw, frame_id: 'map' }) });
          if (r.ok) toast.success('初始位姿已发布'); else toast.error('发布失败');
        } else toast.error('未连接');
      }
    };

    // Mouse: mousedown → mousemove → mouseup
    canvas.addEventListener('mousedown', (e: MouseEvent) => {
      if (!relocalizeModeRef.current && !navGoalModeRef.current) return;
      if (e.button !== 0) return;
      e.stopPropagation(); e.preventDefault();
      const pt = worldPos(e.clientX, e.clientY);
      if (!pt) return;
      estimateStartRef.current = pt.clone();
      estimateModeRef.current = relocalizeModeRef.current ? 'relocalize' : 'navgoal';
      poseEstimateRef.current = { x: pt.x, y: pt.y, yaw: 0 };
      isEstimatingRef.current = true;
      // Red ring marker
      if (markerRef.current && scene) { scene.remove(markerRef.current); markerRef.current.geometry?.dispose(); (markerRef.current.material as any)?.dispose(); }
      const rg = new THREE.RingGeometry(0.3, 0.4, 32);
      markerRef.current = new THREE.Mesh(rg, new THREE.MeshBasicMaterial({ color: 0xff4444, side: THREE.DoubleSide, depthTest: false, transparent: true }));
      markerRef.current.position.set(pt.x, pt.y, 0.5);
      markerRef.current.renderOrder = 10000;
      scene!.add(markerRef.current);
    });
    canvas.addEventListener('mousemove', (e: MouseEvent) => {
      if (!isEstimatingRef.current) return;
      const pt = worldPos(e.clientX, e.clientY);
      if (!pt || !estimateStartRef.current) return;
      updateArrow(estimateStartRef.current, pt);
      if (poseEstimateRef.current) poseEstimateRef.current.yaw = Math.atan2(pt.y - estimateStartRef.current.y, pt.x - estimateStartRef.current.x);
    });

    const finishEstimate = async () => {
      if (!isEstimatingRef.current) return;
      isEstimatingRef.current = false;
      const start = estimateStartRef.current; estimateStartRef.current = null;
      const mode = estimateModeRef.current; estimateModeRef.current = null;
      const pose = poseEstimateRef.current; poseEstimateRef.current = null;
      clearMarker();
      if (start && pose) await publishPose(pose, mode || 'relocalize');
    };
    canvas.addEventListener('mouseup', finishEstimate);

    // Touch: touchstart → touchmove → touchend (mirrors mouse flow)
    const getTouch = (e: TouchEvent) => e.touches[0] || e.changedTouches[0];
    canvas.addEventListener('touchstart', (e: TouchEvent) => {
      if (!relocalizeModeRef.current && !navGoalModeRef.current) return;
      const t = getTouch(e); if (!t || e.touches.length !== 1) return;
      e.stopPropagation(); e.preventDefault();
      const pt = worldPos(t.clientX, t.clientY);
      if (!pt) return;
      estimateStartRef.current = pt.clone();
      estimateModeRef.current = relocalizeModeRef.current ? 'relocalize' : 'navgoal';
      poseEstimateRef.current = { x: pt.x, y: pt.y, yaw: 0 };
      isEstimatingRef.current = true;
      if (markerRef.current && scene) { scene.remove(markerRef.current); markerRef.current.geometry?.dispose(); (markerRef.current.material as any)?.dispose(); }
      const rg = new THREE.RingGeometry(0.3, 0.4, 32);
      markerRef.current = new THREE.Mesh(rg, new THREE.MeshBasicMaterial({ color: 0xff4444, side: THREE.DoubleSide, depthTest: false, transparent: true }));
      markerRef.current.position.set(pt.x, pt.y, 0.5);
      markerRef.current.renderOrder = 10000;
      scene!.add(markerRef.current);
    }, { passive: false });
    canvas.addEventListener('touchmove', (e: TouchEvent) => {
      if (!isEstimatingRef.current) return;
      const t = getTouch(e); if (!t) return;
      e.preventDefault();
      const pt = worldPos(t.clientX, t.clientY);
      if (!pt || !estimateStartRef.current) return;
      updateArrow(estimateStartRef.current, pt);
      if (poseEstimateRef.current) poseEstimateRef.current.yaw = Math.atan2(pt.y - estimateStartRef.current.y, pt.x - estimateStartRef.current.x);
    }, { passive: false });
    canvas.addEventListener('touchend', (e: TouchEvent) => {
      if (!isEstimatingRef.current) return;
      e.stopPropagation();
      finishEstimate();
    });


    console.log('[MapView] Creating LayerManager');
    let layerManager: LayerManager | null = null;
    if (connection) {
      layerManager = new LayerManager(scene, connection);
      layerManagerRef.current = layerManager;
    }

    const handleResize = () => {
      if (!camera || !renderer || !canvas.parentElement) return;
      const width = canvas.parentElement.clientWidth;
      const height = canvas.parentElement.clientHeight;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    };

    window.addEventListener('resize', handleResize);

    const updateRobotPosition = () => {
      const robotConfig = layerConfigsRef.current.robot;
      if (!robotConfig) {
        setRobotPos(null);
        return;
      }

      const baseFrame = (robotConfig as any).baseFrame || 'base_link';
      const mapFrame = (robotConfig as any).mapFrame || 'map';
      const tf2js = TF2JS.getInstance();
      const transform = tf2js.findTransform(mapFrame, baseFrame);

      if (transform) {
        const robotEuler = new THREE.Euler();
        robotEuler.setFromQuaternion(transform.rotation, 'XYZ');
        const robotTheta = robotEuler.z;

        setRobotPos({
          x: transform.translation.x,
          y: transform.translation.y,
          theta: robotTheta,
        });
      } else {
        setRobotPos(null);
      }
    };

    let animationFrameId: number;
    const animate = () => {
      animationFrameId = requestAnimationFrame(animate);
      if (controls && camera) {
        if (focusRobotRef.current) {
          const robotConfig = layerConfigsRef.current.robot;
          if (robotConfig) {
            const baseFrame = (robotConfig as any).baseFrame || 'base_link';
            const mapFrame = (robotConfig as any).mapFrame || 'map';
            const tf2js = TF2JS.getInstance();
            const transform = tf2js.findTransform(mapFrame, baseFrame);
            if (transform) {
              const targetZ = viewModeRef.current === '2d' ? 0 : transform.translation.z;
              controls.target.set(
                transform.translation.x,
                transform.translation.y,
                targetZ
              );
            }
          }
        }

        updateRobotPosition();
        controls.update();
      }
      if (renderer && scene && camera) {
        renderer.render(scene, camera);
      }
    };
    animate();

    return () => {
      window.removeEventListener('resize', handleResize);
      canvas.removeEventListener('click', handleClick);
      cancelAnimationFrame(animationFrameId);
      timeoutRefsRef.current.forEach(timeoutId => clearTimeout(timeoutId));
      timeoutRefsRef.current.clear();
      controls.dispose();
      if (layerManager) layerManager.dispose();
      if (renderer) {
        renderer.dispose();
      }
    };
  }, [connection]);

  useEffect(() => {
    if (!connection?.isConnected()) {
      return;
    }

    const updateRobotPosition = () => {
      const robotConfig = layerConfigsRef.current.robot;
      if (!robotConfig) {
        setRobotPos(null);
        return;
      }

      const baseFrame = (robotConfig as any).baseFrame || 'base_link';
      const mapFrame = (robotConfig as any).mapFrame || 'map';
      const tf2js = TF2JS.getInstance();
      const transform = tf2js.findTransform(mapFrame, baseFrame);

      if (transform) {
        const robotEuler = new THREE.Euler();
        robotEuler.setFromQuaternion(transform.rotation, 'XYZ');
        const robotTheta = robotEuler.z;

        setRobotPos({
          x: transform.translation.x,
          y: transform.translation.y,
          theta: robotTheta,
        });
      } else {
        const availableFrames = tf2js.getFrames();
        if (availableFrames.length > 0) {
          setRobotPos(null);
        }
      }
    };

    const tf2js = TF2JS.getInstance();
    const unsubscribe = tf2js.onTransformChange(() => {
      updateRobotPosition();
    });

    updateRobotPosition();

    const intervalId = setInterval(() => {
      updateRobotPosition();
    }, 100);

    return () => {
      unsubscribe();
      clearInterval(intervalId);
    };
  }, [connection, layerConfigs]);

  useConnectionInit(connection, layerManagerRef, layerConfigs);

  // 通过网关获取地图/机器人/激光数据，渲染到 3D 场景（无需 rosbridge 直连）
  const gatewayCtx = useGatewayContext();
  useGatewaySceneSync({ scene: sceneRef.current, snapshot: gatewayCtx.snapshot, navGoal: navGoal || gatewayCtx.snapshot?.runtime?.navGoal, skipVoxelGrid: connection?.isConnected() ?? false, skipMap: !!gatewayToken });

  // 网关模式下：从遥测 tfPose 更新左下角机器人坐标（rosbridge 直连时 TF2JS 已经更新了）
  useEffect(() => {
    const tfPose = gatewayCtx.snapshot?.runtime?.tfPose;
    if (!tfPose) return;
    // TF2JS 有数据时优先用 TF（更精确），网关 tfPose 作为 fallback
    if (robotPos && connection?.isConnected()) return;
    setRobotPos({
      x: tfPose.x,
      y: tfPose.y,
      theta: tfPose.yaw,
    });
  }, [gatewayCtx.snapshot?.runtime?.tfPose, connection]);

  // 网关模式下，将遥测地图数据注入 MapManager 供 OccupancyGridLayer 渲染
  const lastMapInjectedRef = useRef<string>('');
  useEffect(() => {
    if (!gatewayToken) return;
    const map = gatewayCtx.snapshot?.runtime?.map;
    const mapUpdatedAt = gatewayCtx.snapshot?.runtime?.mapUpdatedAt;
    if (!map || !map.width) return;
    if (mapUpdatedAt && mapUpdatedAt === lastMapInjectedRef.current) return;
    lastMapInjectedRef.current = mapUpdatedAt ?? '';
    const mapUrl = (map as any)?.mapUrl as string | undefined;
    const occ: OccupancyGrid = {
      header: { frame_id: 'map', stamp: { sec: Math.floor(Date.now() / 1000), nsec: 0 } },
      info: {
        map_load_time: { sec: Math.floor(Date.now() / 1000), nsec: 0 },
        resolution: map.resolution,
        width: map.width,
        height: map.height,
        origin: { position: { x: map.origin.x, y: map.origin.y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      },
      data: map.data || [],
      mapUrl: mapUrl || null,
    };
    MapManager.getInstance().updateOccupancyGrid(occ);
  }, [gatewayToken, gatewayCtx.snapshot?.runtime?.map, gatewayCtx.snapshot?.runtime?.mapUpdatedAt]);

  // 多点导航启动时清除旧的 2D Nav Goal
  useEffect(() => {
    if (gatewayCtx.snapshot?.runtime?.navMode === 'multi' && navGoal) {
      setNavGoal(null);
    }
  }, [gatewayCtx.snapshot?.runtime?.navMode]);

  // 系统内存轮询 (5s)
  useEffect(() => {
    const poll = async () => {
      try {
        const res = await fetch('/api/health');
        const d = await res.json() as { system?: { memUsedPercent: number; memFree: number; memTotal: number; heapUsedMB: number; load1m: number } };
        if (d.system) {
          setSysMem({
            usedPercent: d.system.memUsedPercent,
            freeGB: (d.system.memFree / 1024 / 1024 / 1024).toFixed(1),
            totalGB: (d.system.memTotal / 1024 / 1024 / 1024).toFixed(1),
            heapMB: d.system.heapUsedMB,
            load1m: d.system.load1m,
          });
        }
      } catch { /* ignore */ }
    };
    poll();
    const timer = setInterval(poll, 5000);
    return () => clearInterval(timer);
  }, []);

  // 居中视角：地图切换时 / 进入编辑器时
  const centerView = () => {
    const map = gatewayCtx.snapshot?.runtime?.map;
    if (!map || map.width === 0 || !cameraRef.current || !controlsRef.current) return;
    const cx = map.origin.x + (map.width * map.resolution) / 2;
    const cy = map.origin.y + (map.height * map.resolution) / 2;
    const mapSize = Math.max(map.width * map.resolution, map.height * map.resolution);
    const dist = mapSize * 1.2;
    controlsRef.current.target.set(cx, cy, 0);
    cameraRef.current.position.set(cx, cy, dist);
    cameraRef.current.lookAt(cx, cy, 0);
    controlsRef.current.update();
  };
  const lastCenteredMapRef = useRef<string>('');
  useEffect(() => {
    const map = gatewayCtx.snapshot?.runtime?.map;
    if (!map || map.width === 0) return;
    const mapKey = `${map.width}x${map.height}@${map.origin.x},${map.origin.y}`;
    if (mapKey === lastCenteredMapRef.current) return;
    lastCenteredMapRef.current = mapKey;
    centerView();
  }, [gatewayCtx.snapshot?.runtime?.map?.width, gatewayCtx.snapshot?.runtime?.map?.height]);
  useEffect(() => {
    if (!showEditor) return;
    // Retry up to 2s if map data hasn't arrived yet
    let tries = 0;
    const timer = setInterval(() => {
      const map = gatewayCtx.snapshot?.runtime?.map;
      if (map && map.width > 0 && cameraRef.current && controlsRef.current) {
        centerView();
        clearInterval(timer);
      }
      if (++tries > 20) clearInterval(timer);
    }, 100);
    return () => clearInterval(timer);
  }, [showEditor]);

  useLayerConfigSync(
    layerConfigs,
    layerConfigsRef,
    layerManagerRef,
    connection,
    initialposeTopicRef
  );


  useEffect(() => {
    focusRobotRef.current = focusRobot;
  }, [focusRobot]);

  // 为 Gateway 面板添加原生触摸滚动支持（绕过浏览器 passive 事件限制）
  useEffect(() => {
    const panel = gwPanelRef.current;
    if (!panel) return;
    let startY = 0;
    let startScroll = 0;
    const handleTouchStart = (e: TouchEvent) => {
      if (e.touches.length !== 1) return;
      startY = e.touches[0].clientY;
      startScroll = panel.scrollTop;
    };
    const handleTouchMove = (e: TouchEvent) => {
      if (e.touches.length !== 1) return;
      const dy = startY - e.touches[0].clientY;
      panel.scrollTop = startScroll + dy;
    };
    panel.addEventListener('touchstart', handleTouchStart, { passive: true });
    panel.addEventListener('touchmove', handleTouchMove, { passive: true });
    return () => {
      panel.removeEventListener('touchstart', handleTouchStart);
      panel.removeEventListener('touchmove', handleTouchMove);
    };
  }, [gatewayToken, gatewayPanel]);

  const handleViewModeToggle = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    e.stopPropagation();
    setViewMode((prev) => {
      const newMode = prev === '2d' ? '3d' : '2d';
      viewModeRef.current = newMode;
      console.log(`切换视图模式: ${prev} -> ${newMode}`);
      return newMode;
    });
  };

  const handleFocusRobotToggle = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    e.stopPropagation();
    setFocusRobot((prev) => !prev);
  };

  const handleDensityChange = (value: number) => {
    setPcDensity(value);
    setLayerConfigs((prev) => {
      const next = { ...prev };
      if (next.map_cloud) next.map_cloud = { ...next.map_cloud, maxTotalPoints: value };
      return next;
    });
  };

  const handleScanDistanceChange = (value: number) => {
    setScanDistance(value);
    setLayerConfigs((prev) => {
      const next = { ...prev };
      const maxDist = value >= 5 ? Infinity : value;
      if (next.map_cloud) next.map_cloud = { ...next.map_cloud, maxDistance: maxDist };
      if (next.laser_map) next.laser_map = { ...next.laser_map, maxDistance: maxDist };
      if (next.voxel_grid) next.voxel_grid = { ...next.voxel_grid, maxDistance: maxDist };
      return next;
    });
  };

  const handleNavToleranceChange = async (value: number) => {
    setNavTolerance(value);
    if (!gatewayTokenRef.current) return;
    try {
      const { apiBase } = await import('../api/gatewayApi');
      await fetch(`${apiBase}/api/nav/tolerance`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', Authorization: `Bearer ${gatewayTokenRef.current}` },
        body: JSON.stringify({ xy_tolerance: value }),
      });
    } catch {}
  };

  const handleObstacleDistChange = async (value: number) => {
    setObstacleDist(value);
    if (!gatewayTokenRef.current) return;
    try {
      const { apiBase } = await import('../api/gatewayApi');
      await fetch(`${apiBase}/api/nav/obstacle-dist`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', Authorization: `Bearer ${gatewayTokenRef.current}` },
        body: JSON.stringify({ min_dist: value }),
      });
    } catch {}
  };

  const exitEstimateMode = () => {
    isEstimatingRef.current = false;
    estimateModeRef.current = null;
    if (estimateLineObjRef.current && sceneRef.current) {
      sceneRef.current.remove(estimateLineObjRef.current);
      estimateLineObjRef.current.geometry?.dispose();
      (estimateLineObjRef.current.material as THREE.Material)?.dispose();
      estimateLineObjRef.current = null;
    }
    if (controlsRef.current) {
      controlsRef.current.enabled = true;
      controlsRef.current.enablePan = true;
      controlsRef.current.enableRotate = viewModeRef.current === '3d';
      controlsRef.current.enableZoom = true;
    }
  };

  const enterEstimateMode = (mode: 'relocalize' | 'navgoal') => {
    const isAlready = (mode === 'relocalize' && relocalizeModeRef.current) || (mode === 'navgoal' && navGoalModeRef.current);
    // 先退出当前模式
    if (navGoalMode) { setNavGoalMode(false); navGoalModeRef.current = false; }
    if (relocalizeMode) { setRelocalizeMode(false); relocalizeModeRef.current = false; }
    exitEstimateMode();
    if (isAlready) return; // 点同一个按钮 = 退出
    // 进入新模式
    if (mode === 'relocalize') { setRelocalizeMode(true); relocalizeModeRef.current = true; }
    else { setNavGoalMode(true); navGoalModeRef.current = true; }
    if (controlsRef.current) {
      controlsRef.current.enablePan = false;
      controlsRef.current.enableRotate = false;
      controlsRef.current.enableZoom = false;
      controlsRef.current.enabled = false; // 完全禁用 OrbitControls，避免 pointer capture 吃掉 touch 事件
    }
    if (viewMode !== '2d') { setViewMode('2d'); viewModeRef.current = '2d'; }
  };

  const handleRelocalizeToggle = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault(); e.stopPropagation();
    enterEstimateMode('relocalize');
  };
  const handleNavGoalToggle = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault(); e.stopPropagation();
    enterEstimateMode('navgoal');
  };

  const handleGatewayLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      const { apiBase } = await import('../api/gatewayApi');
      const res = await fetch(apiBase + '/api/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username: gwUsername, password: gwPassword }),
      });
      if (!res.ok) throw new Error('登录失败');
      const body = await res.json() as { token: string };
      onGatewayLogin(body.token);
      setShowGatewayLogin(false);
      setGwLoginError('');
    } catch (e) {
      setGwLoginError(String(e));
    }
  };



  return (
    <div className="MapView">
      {/* Top bar: gateway */}
      <div className="MapViewTopBar">
        {/* Gateway section */}
        <div className="TopBarSection">
          {gatewayToken ? (
            <>
              <span style={{ color: '#4CAF50', fontSize: '13px', whiteSpace: 'nowrap', flexShrink: 0 }}>● 网关</span>
              <span
                title={`系统: ${sysMem.freeGB}/${sysMem.totalGB}GB 空闲 | 进程: ${sysMem.heapMB}MB | 负载: ${sysMem.load1m}`}
                style={{
                  color: sysMem.usedPercent > 85 ? '#e94560' : sysMem.usedPercent > 70 ? '#FF9800' : '#4CAF50',
                  fontSize: '12px',
                  whiteSpace: 'nowrap',
                  flexShrink: 0,
                  marginLeft: '8px',
                  cursor: 'default',
                }}
              >
                █ {sysMem.usedPercent}%
              </span>
              <div className="gateway-buttons-inline">
              {(() => {
                const r = gatewayCtx.snapshot?.runtime;
                const statusColor = (id: string) => {
                  let status = '';
                  if (id === 'mapping') status = r?.mappingStatus ?? '';
                  else if (id === 'lidar-control') status = r?.lidarStatus ?? '';
                  else if (id === 'motion-control') status = r?.motionStatus ?? '';
                  else if (id === 'nav-control') status = r?.navStatus ?? '';
                  if (status === 'running') return '#4CAF50';
                  if (status === 'paused') return '#FF9800';
                  if (status === 'stopped') return '#FF9800';
                  return '#666';
                };
                return gatewayButtons.map((btn) => (
                  <button
                    key={btn.id}
                    className={'TopBarBtn' + (gatewayPanel === btn.id ? ' active' : '')}
                    onClick={() => setGatewayPanel(gatewayPanel === btn.id ? null : btn.id)}
                    title={btn.label}
                    type="button"
                    style={{ borderColor: statusColor(btn.id), borderWidth: '2px', borderStyle: 'solid' }}
                  >
                    <span className="gw-btn-icon">{btn.icon}</span>
                    <span className="gw-btn-label">{btn.label}</span>
                  </button>
                ));
              })()}
              </div>
              <button
                className="TopBarBtn"
                onClick={onGatewayLogout}
                title="退出网关"
                type="button"
              >
                <span className="gw-btn-icon">🚪</span>
                <span className="gw-btn-label">退出</span>
              </button>
            </>
          ) : (
            <button
              className="TopBarBtn TopBarBtnGateway"
              onClick={() => setShowGatewayLogin(true)}
              type="button"
            >
              🔐 网关登录
            </button>
          )}
        </div>
      </div>


      {/* Gateway login dialog */}
      {showGatewayLogin && (
        <div className="GatewayLoginOverlay" onClick={() => setShowGatewayLogin(false)}>
          <form className="GatewayLoginForm" onClick={(e) => e.stopPropagation()} onSubmit={handleGatewayLogin}>
            <h3 style={{ marginTop: 0, textAlign: 'center', color: '#fff' }}>网关登录</h3>
            <div style={{ marginBottom: '12px' }}>
              <label style={{ display: 'block', marginBottom: '4px', fontSize: '13px', color: '#aaa' }}>用户名</label>
              <input
                value={gwUsername}
                onChange={(e) => setGwUsername(e.target.value)}
                style={{
                  width: '100%', padding: '8px', borderRadius: '4px', border: '1px solid #333',
                  backgroundColor: '#0f3460', color: '#eee', boxSizing: 'border-box',
                }}
              />
            </div>
            <div style={{ marginBottom: '16px' }}>
              <label style={{ display: 'block', marginBottom: '4px', fontSize: '13px', color: '#aaa' }}>密码</label>
              <input
                value={gwPassword}
                onChange={(e) => setGwPassword(e.target.value)}
                type="password"
                style={{
                  width: '100%', padding: '8px', borderRadius: '4px', border: '1px solid #333',
                  backgroundColor: '#0f3460', color: '#eee', boxSizing: 'border-box',
                }}
              />
            </div>
            <button
              type="submit"
              style={{
                width: '100%', padding: '10px', borderRadius: '4px', border: 'none',
                backgroundColor: '#e94560', color: 'white', fontSize: '14px', cursor: 'pointer', fontWeight: 'bold',
              }}
            >
              登录
            </button>
            {gwLoginError && <p style={{ color: '#e94560', fontSize: '12px', marginTop: '8px', textAlign: 'center' }}>{gwLoginError}</p>}
          </form>
        </div>
      )}

      {/* Gateway panel overlay */}
      {gatewayToken && gatewayPanel && (
        <div className="GatewayPanelOverlay" ref={gwPanelRef}>
          <div className="GatewayPanelHeader">
            <h3>{gatewayButtons.find(b => b.id === gatewayPanel)?.label || ''}</h3>
            <button
              className="GatewayPanelClose"
              onClick={() => setGatewayPanel(null)}
              type="button"
            >
              ×
            </button>
          </div>
          <div className="GatewayPanelContent">
            <GatewayPanelRenderer panel={gatewayPanel} />
          </div>
        </div>
      )}

      <div className="ViewControls">
        <button
          className={`ViewButton ${viewMode === '2d' ? 'active' : ''}`}
          onClick={handleViewModeToggle}
          title={`当前: ${viewMode === '2d' ? '2D' : '3D'}视图，点击切换到${viewMode === '2d' ? '3D' : '2D'}`}
          type="button"
        >
          {viewMode === '2d' ? '2D' : '3D'}
        </button>
        <button
          className="SettingsButton"
          onClick={() => setShowEditor(true)}
          title="地图编辑"
          type="button"
        >
          ✏️
        </button>
        <button
          className={`SettingsButton ${showNavParams ? 'active' : ''}`}
          onClick={() => setShowNavParams(prev => !prev)}
          title="导航参数配置"
          type="button"
        >
          ⚙️
        </button>
        <button
          className={`SettingsButton ${relocalizeMode ? 'active' : ''}`}
          onClick={handleRelocalizeToggle}
          title={relocalizeMode ? '退出重定位' : '重定位 (2D Pose Estimate)'}
          type="button"
        >
          📍
        </button>
        <button
          className={`SettingsButton ${navGoalMode ? 'active' : ''}`}
          onClick={handleNavGoalToggle}
          title={navGoalMode ? '退出导航目标' : gatewayCtx.snapshot?.runtime?.navMode === 'multi' ? '多点导航中不可用' : '导航目标 (2D Nav Goal)'}
          type="button"
          disabled={gatewayCtx.snapshot?.runtime?.navMode === 'multi'}
          style={gatewayCtx.snapshot?.runtime?.navMode === 'multi' ? { opacity: 0.3, cursor: 'not-allowed' } : undefined}
        >
          🎯
        </button>
      </div>
      {(relocalizeMode || navGoalMode) && (
        <div style={{
          position: 'absolute', bottom: '60px', left: '50%', transform: 'translateX(-50%)',
          backgroundColor: 'rgba(233,69,96,0.9)', color: '#fff', padding: '6px 16px',
          borderRadius: '6px', fontSize: '13px', zIndex: 100,
        }}>
          {relocalizeMode ? '📍 重定位：' : '🎯 导航目标：'}点住地图放置，拖拽设朝向，松开即发布
        </div>
      )}
      <div className="BottomControls">
        <button
          className={`FocusRobotButton ${focusRobot ? 'active' : ''}`}
          onClick={handleFocusRobotToggle}
          title={focusRobot ? '取消跟随机器人' : '跟随机器人'}
          type="button"
        >
          {focusRobot ? '📍 跟随中' : '📍 跟随机器人'}
        </button>
      </div>
      {showEditor && (
        <MapEditor
          connection={connection}
          gatewayToken={gatewayToken}
          onClose={() => setShowEditor(false)}
        />
      )}
      {showNavParams && (
        <NavParamsPanel
          gatewayToken={gatewayToken}
          onClose={() => setShowNavParams(false)}
          pcDensity={pcDensity}
          scanDistance={scanDistance}
          onPcDensityChange={(v) => { setPcDensity(v); handleDensityChange(v); }}
          onScanDistanceChange={(v) => { setScanDistance(v); handleScanDistanceChange(v); }}
        />
      )}
      <TopoPointInfoPanel
        selectedPoint={selectedTopoPoint}
        selectedRoute={selectedTopoRoute}
        onClose={() => {
          setSelectedTopoPoint(null);
          setSelectedTopoRoute(null);
          const topoLayer = layerManagerRef.current?.getLayer('topology');
          if (topoLayer && 'setSelectedPoint' in topoLayer) {
            (topoLayer as any).setSelectedPoint(null);
          }
          if (topoLayer && 'setSelectedRoute' in topoLayer) {
            (topoLayer as any).setSelectedRoute(null);
          }
        }}
        connection={connection}
      />
      <canvas ref={canvasRef} className="MapCanvas" style={{ touchAction: 'none' }}
        onMouseMove={handleCanvasMouseMove}
        onTouchStart={handleCanvasTouch}
        onTouchMove={handleCanvasTouch}
      />
      {Array.from(imageLayers.entries())
        .filter(([layerId]) => layerConfigs[layerId]?.enabled)
        .map(([layerId, imageData]) => {
          const config = layerConfigs[layerId];
          const position = imagePositionsRef.current.get(layerId) || { x: 100, y: 100, scale: 1 };
          return (
            <ImageDisplay
              key={layerId}
              imageData={imageData}
              name={config?.name || layerId}
              position={position}
              onPositionChange={(newPos) => {
                imagePositionsRef.current.set(layerId, newPos);
                const positionsMap: ImagePositionsMap = {};
                imagePositionsRef.current.forEach((pos, id) => {
                  positionsMap[id] = pos;
                });
                saveImagePositions(positionsMap);
              }}
            />
          );
        })}
      <div className="CoordinateDisplay">
        <div className="CoordinateRow">
          <span className="CoordinateLabel">鼠标:</span>
          <span className="CoordinateValue">
            {mouseWorldPos
              ? `X: ${mouseWorldPos.x.toFixed(3)}, Y: ${mouseWorldPos.y.toFixed(3)}`
              : '-'}
          </span>
        </div>
        {relocalizeMode ? (
          <div className="CoordinateRow">
            <span className="CoordinateLabel">重定位:</span>
            <span className="CoordinateValue">
              {poseEstimateRef.current
                ? `点住拖拽设定位置和朝向`
                : '在地图上点住拖拽'}
            </span>
          </div>
        ) : (
          <div className="CoordinateRow">
            <span className="CoordinateLabel">机器人:</span>
            <span className="CoordinateValue">
              {robotPos
                ? `X: ${robotPos.x.toFixed(3)}, Y: ${robotPos.y.toFixed(3)}, θ: ${robotPos.theta.toFixed(3)}`
                : '-'}
            </span>
          </div>
        )}
      </div>
    </div>
  );
}
