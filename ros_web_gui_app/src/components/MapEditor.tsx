import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { toast } from 'react-toastify';
import { RosbridgeConnection } from '../utils/RosbridgeConnection';
import { TF2JS } from '../utils/tf2js';
import { LayerManager } from './layers/LayerManager';
import type { LayerConfigMap } from '../types/LayerConfig';
import { TopoLayer } from './layers/TopoLayer';
import { OccupancyGridLayer } from './layers/OccupancyGridLayer';
import { MapManager } from '../utils/MapManager';
import type { TopoPoint, OccupancyGrid } from '../utils/MapManager';
import {
  CommandManager,
  AddPointCommand,
  DeletePointCommand,
  ModifyPointCommand,
  ModifyGridCommand,
  type GridCellChange,
} from '../utils/CommandManager';
import { publishGridToMap } from './MapEditor.GatewaySave';
import { useGatewayContext } from './gateway/GatewayProvider';
import './MapEditor.css';

interface MapEditorProps {
  connection: RosbridgeConnection | null;
  onClose: () => void;
  gatewayToken?: string | null;
}

type EditTool = 'move' | 'addPoint' | 'brush' | 'eraser' | 'drawLine' | 'rect' | 'circle' | 'rotateView';

const DEFAULT_EDITOR_CONFIGS: LayerConfigMap = {
  occupancy_grid: {
    id: 'occupancy_grid',
    name: '栅格地图',
    topic: '/map',
    messageType: 'nav_msgs/OccupancyGrid',
    enabled: true,
    colorMode: 'map',
    height: 0,
  },
  topology: {
    id: 'topology',
    name: 'Topo地图',
    topic: '/map/topology',
    messageType: null,
    enabled: true,
    color: 0x2196f3,
    pointSize: 0.2,
  },
  teb_local_plan: {
    id: 'teb_local_plan',
    name: 'TEB局部路径',
    topic: '/move_base/TebLocalPlannerROS/local_plan',
    messageType: 'nav_msgs/Path',
    enabled: true,
    color: 0x00ff00,
    lineWidth: 2,
  },
  global_plan: {
    id: 'global_plan',
    name: '全局规划路径',
    topic: '/move_base/GlobalPlanner/plan',
    messageType: 'nav_msgs/Path',
    enabled: true,
    color: 0x0000ff,
    lineWidth: 3,
  },
  voxel_grid: {
    id: 'pointcloud2',
    name: '体素障碍物点云',
    topic: '/move_base/local_costmap/stvl_obstacle_layer/voxel_grid',
    messageType: 'sensor_msgs/PointCloud2',
    enabled: true,
    color: 0xff6600,
    pointSize: 0.05,
    targetFrame: 'map',
  },
};

export function MapEditor({ connection, onClose, gatewayToken }: MapEditorProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const layerManagerRef = useRef<LayerManager | null>(null);
  const topoLayerRef = useRef<TopoLayer | null>(null);
  const occupancyGridLayerRef = useRef<OccupancyGridLayer | null>(null);
  const [currentTool, setCurrentTool] = useState<EditTool>('move');
  const [brushSize, setBrushSize] = useState<number>(0.05);
  const [mousePosition, setMousePosition] = useState<{ x: number; y: number } | null>(null);
  const lastDrawPosRef = useRef<THREE.Vector3 | null>(null);
  const brushIndicatorRef = useRef<HTMLDivElement | null>(null);
  const isDrawingRef = useRef(false);
  const drawRafIdRef = useRef<number | null>(null);
  const [selectedPoint, setSelectedPoint] = useState<TopoPoint | null>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [isRotating, setIsRotating] = useState(false);
  const [dragStartPos, setDragStartPos] = useState<THREE.Vector2 | null>(null);
  const [lineStartPoint, setLineStartPoint] = useState<THREE.Vector3 | null>(null);
  const raycasterRef = useRef<THREE.Raycaster | null>(null);
  const mapManagerRef = useRef<MapManager>(MapManager.getInstance());
  const commandManagerRef = useRef<CommandManager>(new CommandManager());
  const selectedPointRef = useRef<THREE.Group | null>(null);
  const previewLineRef = useRef<THREE.Line | null>(null);
  const shapePreviewRef = useRef<THREE.Mesh | null>(null);
  const shapeBorderRef = useRef<THREE.Line | null>(null);
  const shapeStartRef = useRef<THREE.Vector3 | null>(null);
  const shapeEndRef = useRef<THREE.Vector3 | null>(null);
  const isPlacingShapeRef = useRef(false);
  const selectedPointStateRef = useRef<TopoPoint | null>(null);
  const currentGridChangesRef = useRef<Map<number, { oldValue: number; newValue: number }>>(new Map());
  const timeoutRefsRef = useRef<Set<ReturnType<typeof setTimeout>>>(new Set());
  const initialGridValuesRef = useRef<Map<number, number>>(new Map());
  const dragStartPointRef = useRef<TopoPoint | null>(null);
  const yawAngleRef = useRef<number>(0);               // yaw 旋转角度 (弧度)
  const isYawRotatingRef = useRef<boolean>(false);
  const yawStartMouseRef = useRef<{ x: number; y: number; yaw: number } | null>(null);
  const isPlacingPointRef = useRef(false);
  const placingPointStartRef = useRef<THREE.Vector3 | null>(null);
  const placingPointInitialRef = useRef<TopoPoint | null>(null);
  const placingPointArrowRef = useRef<THREE.Line | null>(null);
  const [supportControllers, setSupportControllers] = useState<string[]>(['FollowPath']);
  const [supportGoalCheckers, setSupportGoalCheckers] = useState<string[]>(['general_goal_checker']);
  const [mouseWorldPos, setMouseWorldPos] = useState<{ x: number; y: number } | null>(null);
  const [robotPos, setRobotPos] = useState<{ x: number; y: number; theta: number } | null>(null);
  const [editingPoint, setEditingPoint] = useState<TopoPoint | null>(null);
  const isTouchSessionRef = useRef(false); // 触摸会话标记，避免 mousemove 误判

  // 通过网关获取栅格地图数据，注入 MapManager 供 OccupancyGridLayer 使用
  // 仅在 mapUpdatedAt 变化时注入，避免遥测每 1s 推送覆盖用户编辑
  const gatewayCtx = useGatewayContext();
  const lastMapUpdatedRef = useRef<string | null>(null);

  /** Fetch PGM binary, parse into Int8Array with Y-flip.
   *  PGM rows are top→bottom; OccupancyGrid rows are bottom→top (origin at bottom-left). */
  async function fetchPgmData(pgmUrl: string, width: number, height: number): Promise<Int8Array> {
    const resp = await fetch(pgmUrl)
    if (!resp.ok) throw new Error(`PGM fetch failed: ${resp.status}`)
    const buf = await resp.arrayBuffer()
    const bytes = new Uint8Array(buf)
    // Skip PGM header — handle comment lines (#) correctly
    // PGM format: P5\n [#comments\n]* <width> <height>\n 255\n <pixel data>
    let hdrEnd = 0
    const readLineEnd = (): number => {
      while (hdrEnd < bytes.length && bytes[hdrEnd] !== 10) hdrEnd++
      return hdrEnd++ // return position of \n, then skip it
    }
    readLineEnd() // skip P5 line
    let nextLineStart = hdrEnd
    while (bytes[nextLineStart] === 0x23 /* # */ || bytes[nextLineStart] === 10) {
      if (bytes[nextLineStart] === 10) { hdrEnd++; nextLineStart++; continue }
      readLineEnd() // skip comment line
      nextLineStart = hdrEnd
    }
    readLineEnd() // skip dimensions line
    // Skip optional comment lines before maxval and the maxval line itself
    nextLineStart = hdrEnd
    while (bytes[nextLineStart] === 0x23 /* # */ || bytes[nextLineStart] === 10) {
      if (bytes[nextLineStart] === 10) { hdrEnd++; nextLineStart++; continue }
      readLineEnd()
      nextLineStart = hdrEnd
    }
    readLineEnd() // skip maxval line (255)
    // hdrEnd now points to first pixel byte
    const data = new Int8Array(width * height)
    // Flip Y: PGM row 0 (top) → OccupancyGrid row (height-1) (top)
    for (let y = 0; y < height; y++) {
      const srcOff = hdrEnd + y * width
      const dstOff = (height - 1 - y) * width
      for (let x = 0; x < width; x++) {
        const v = bytes[srcOff + x]!
        data[dstOff + x] = v <= 1 ? 100 : v >= 250 ? 0 : -1
      }
    }
    return data
  }

  useEffect(() => {
    const map = gatewayCtx.snapshot?.runtime?.map;
    const mapUpdatedAt = gatewayCtx.snapshot?.runtime?.mapUpdatedAt;
    const hasDirectData = map?.data && map.data.length > 0;
    const mapUrl = (map as any)?.mapUrl as string | undefined;
    if (!map || (!hasDirectData && !mapUrl)) return;
    if (mapUpdatedAt && mapUpdatedAt === lastMapUpdatedRef.current) return;
    lastMapUpdatedRef.current = mapUpdatedAt ?? null;

    const inject = (data: number[] | Int8Array) => {
      const occ: OccupancyGrid = {
        header: { frame_id: 'map', stamp: { sec: Math.floor(Date.now() / 1000), nsec: 0 } },
        info: {
          map_load_time: { sec: Math.floor(Date.now() / 1000), nsec: 0 },
          resolution: map.resolution,
          width: map.width,
          height: map.height,
          origin: { position: { x: map.origin.x, y: map.origin.y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
        },
        data,
        mapUrl: mapUrl || null,
      };
      mapManagerRef.current.updateOccupancyGrid(occ);
    };

    if (hasDirectData) {
      inject(map.data);
    } else if (mapUrl) {
      // Large map: fetch PGM binary for editing, while display uses cached PNG
      const pgmUrl = mapUrl.replace('live_map.png', 'live_map.pgm').replace(/\?t=\d+/, '');
      fetchPgmData(pgmUrl, map.width, map.height)
        .then((data) => inject(data))
        .catch((err) => console.error('[MapEditor] PGM fetch for editing failed:', err));
    }
  }, [gatewayCtx.snapshot?.runtime?.map, gatewayCtx.snapshot?.runtime?.mapUpdatedAt]);

  useEffect(() => {
    if (!canvasRef.current) return;

    const canvas = canvasRef.current;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a1a);
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

    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    rendererRef.current = renderer;

    const controls = new OrbitControls(camera, canvas);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = true;
    controls.minDistance = 0.1;
    controls.maxDistance = 1000;
    controls.target.set(0, 0, 0);
    controls.mouseButtons.LEFT = THREE.MOUSE.PAN;
    controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
    controls.mouseButtons.MIDDLE = THREE.MOUSE.ROTATE;
    (controls as OrbitControls & { zoomToCursor?: boolean }).zoomToCursor = true;
    // 平板触摸手势：单指平移，双指旋转+缩放
    (controls as any).touches = {
      ONE: THREE.TOUCH.PAN,
      TWO: THREE.TOUCH.DOLLY_ROTATE,
    };

    // 2D 模式：禁用 OrbitControls 旋转，yaw 旋转由旋转视角工具处理
    controls.enableRotate = false;
    controls.enableZoom = true;
    controls.enablePan = true;
    camera.up.set(0, 0, 1);
    camera.position.set(0, 0, 10);
    controls.update();  // OrbitControls 自动将相机对准 target 产生俯视姿态
    controlsRef.current = controls;

    const raycaster = new THREE.Raycaster();
    raycasterRef.current = raycaster;

    const layerManager = new LayerManager(scene, connection);
    layerManagerRef.current = layerManager;

    // 获取 topology layer 引用
    const topoLayer = layerManager.getLayer('topology') as TopoLayer | undefined;
    if (topoLayer) {
      topoLayerRef.current = topoLayer;
    }

    // 获取 occupancy_grid layer 引用
    const occupancyGridLayer = layerManager.getLayer('occupancy_grid');
    if (occupancyGridLayer instanceof OccupancyGridLayer) {
      occupancyGridLayerRef.current = occupancyGridLayer;
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
      const tf2js = TF2JS.getInstance();
      const transform = tf2js.findTransform('map', 'base_link');

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

    let animationFrameId: number;
    const animate = () => {
      animationFrameId = requestAnimationFrame(animate);
      if (controls && camera) {
        camera.up.set(0, 0, 1);
        controls.update();
        // 确保相机在目标正上方（俯视）
        const targetZ = 0;
        const distance = Math.max(Math.abs(camera.position.z - targetZ), controls.minDistance);
        camera.position.set(controls.target.x, controls.target.y, targetZ + distance);
        // 在 OrbitControls 俯视姿态上叠加世界 Z 轴 yaw 旋转
        const yawQuat = new THREE.Quaternion().setFromAxisAngle(
          new THREE.Vector3(0, 0, 1),
          yawAngleRef.current
        );
        camera.quaternion.premultiply(yawQuat);
      }
      if (renderer && scene && camera) {
        renderer.render(scene, camera);
      }
      updateRobotPosition();
    };
    animate();

    return () => {
      window.removeEventListener('resize', handleResize);
      cancelAnimationFrame(animationFrameId);
      timeoutRefsRef.current.forEach(timeoutId => clearTimeout(timeoutId));
      timeoutRefsRef.current.clear();
      clearPreviewLine();
      clearShapePreview();
      controls.dispose();
      layerManager.dispose();
      if (renderer) {
        renderer.dispose();
      }
    };
  }, [connection]);

  useEffect(() => {
    if (!layerManagerRef.current) {
      return;
    }

    // 网关模式（无 rosbridge 直连）：只创建不需要 rosbridge 的图层
    if (!connection || !connection.isConnected()) {
      layerManagerRef.current.setLayerConfigs({
        occupancy_grid: DEFAULT_EDITOR_CONFIGS.occupancy_grid!,
        topology: DEFAULT_EDITOR_CONFIGS.topology!,
      });
      setTimeout(() => {
        const topoLayer = layerManagerRef.current?.getLayer('topology') as TopoLayer | undefined;
        if (topoLayer) {
          topoLayerRef.current = topoLayer;
          updateTopoMap();
        }
        const occupancyGridLayer = layerManagerRef.current?.getLayer('occupancy_grid');
        if (occupancyGridLayer instanceof OccupancyGridLayer) {
          occupancyGridLayerRef.current = occupancyGridLayer;
        }
      }, 500);
      return;
    }

    let timeoutId: ReturnType<typeof setTimeout> | null = null;
    let handleMapUpdate: (() => void) | null = null;

    const initializeAndSubscribe = async () => {
      try {
        await connection.initializeMessageReaders();
        TF2JS.getInstance().initialize(connection);
        layerManagerRef.current?.setLayerConfigs(DEFAULT_EDITOR_CONFIGS);

        const mapManager = mapManagerRef.current;
        mapManager.initialize(connection);

        handleMapUpdate = () => {
          updateTopoMap();
        };
        mapManager.addTopologyListener(handleMapUpdate);

        timeoutId = setTimeout(() => {
          const topoLayer = layerManagerRef.current?.getLayer('topology') as TopoLayer | undefined;
          if (topoLayer) {
            topoLayerRef.current = topoLayer;
            updateTopoMap();
          }

          const occupancyGridLayer = layerManagerRef.current?.getLayer('occupancy_grid');
          if (occupancyGridLayer instanceof OccupancyGridLayer) {
            occupancyGridLayerRef.current = occupancyGridLayer;
          }
        }, 500);
      } catch (error) {
        console.warn('Failed to initialize message readers, using default configs:', error);
        TF2JS.getInstance().initialize(connection);
        layerManagerRef.current?.setLayerConfigs(DEFAULT_EDITOR_CONFIGS);
      }
    };

    void initializeAndSubscribe();

    return () => {
      if (timeoutId) {
        clearTimeout(timeoutId);
      }
      if (handleMapUpdate && mapManagerRef.current) {
        mapManagerRef.current.removeTopologyListener(handleMapUpdate);
      }
      mapManagerRef.current.disconnect();
    };
  }, [connection]);

  useEffect(() => {
    if (!connection || !connection.isConnected()) {
      return;
    }

    const updateRobotPosition = () => {
      const tf2js = TF2JS.getInstance();
      const transform = tf2js.findTransform('map', 'base_link');

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
  }, [connection]);

  const getWorldPosition = (event: MouseEvent | { clientX: number; clientY: number }): THREE.Vector3 | null => {
    if (!cameraRef.current || !canvasRef.current) return null;

    const rect = canvasRef.current.getBoundingClientRect();
    const mouse = new THREE.Vector2();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    const raycaster = raycasterRef.current;
    if (!raycaster) return null;

    raycaster.setFromCamera(mouse, cameraRef.current);
    const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    const intersectPoint = new THREE.Vector3();
    raycaster.ray.intersectPlane(plane, intersectPoint);

    return intersectPoint;
  };

  const handleCanvasClick = (event: MouseEvent) => {
    if (!sceneRef.current || !cameraRef.current || isDragging || isRotating || isYawRotatingRef.current) return;

    const rect = canvasRef.current!.getBoundingClientRect();
    const mouse = new THREE.Vector2();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycasterRef.current!.setFromCamera(mouse, cameraRef.current);
    const intersects = raycasterRef.current!.intersectObjects(sceneRef.current.children, true);

    if (currentTool === 'drawLine') {
      // 直线绘制工具：点击空白区域设置起始点和结束点
      const worldPos = getWorldPosition(event);
      if (!worldPos || !occupancyGridLayerRef.current) return;

      if (!lineStartPoint) {
        setLineStartPoint(worldPos);
        createPreviewLine(worldPos.x, worldPos.y, worldPos.x, worldPos.y);
        toast.info('已设置起始点，点击设置结束点');
      } else {
        const endPos = worldPos;
        const changes = occupancyGridLayerRef.current.drawLine(
          lineStartPoint.x,
          lineStartPoint.y,
          endPos.x,
          endPos.y,
          100,
          brushSize
        );

        if (changes.length > 0) {
          const command = new ModifyGridCommand(
            occupancyGridLayerRef.current,
            changes,
            () => { }
          );
          commandManagerRef.current.executeCommand(command);
        }

        clearPreviewLine();
        setLineStartPoint(null);
        toast.success('已绘制直线');
      }
      return;
    }

    // 检查是否点击了点位
    for (const intersect of intersects) {
      let obj = intersect.object;
      while (obj) {
        if (obj.userData.isTopoPoint && obj.userData.topoPoint) {
          const point = obj.userData.topoPoint;
          const mapManager = mapManagerRef.current;
          const pointData = mapManager.getTopologyPoint(point.name);
          if (pointData) {
            setSelectedPoint(pointData);
            const topoLayer = layerManagerRef.current?.getLayer('topology');
            if (topoLayer instanceof TopoLayer) {
              topoLayer.setSelectedPoint(pointData);
            }
            return;
          }
        }
        obj = obj.parent as THREE.Object3D;
      }
    }

  };

  const handleCanvasMouseDown = (event: MouseEvent) => {
    // 添加点位工具：mousedown 放置 + 拖拽设方向 + mouseup 确认
    if (currentTool === 'addPoint' && event.button === 0) {
      // 先检查是否点击了已有点位（命中则交由 click 处理选中）
      const rect = canvasRef.current!.getBoundingClientRect();
      const mouse = new THREE.Vector2();
      mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycasterRef.current!.setFromCamera(mouse, cameraRef.current!);
      const intersects = raycasterRef.current!.intersectObjects(sceneRef.current!.children, true);
      for (const intersect of intersects) {
        let obj = intersect.object;
        while (obj) {
          if (obj.userData.isTopoPoint) {
            // 已有点位，不创建新点位，交给 click handler 处理选中
            return;
          }
          obj = obj.parent as THREE.Object3D;
        }
      }
      // 空白区域：创建新点位
      event.preventDefault();
      event.stopPropagation();
      if (controlsRef.current) {
        controlsRef.current.enablePan = false;
        controlsRef.current.enableRotate = false;
      }
      const worldPos = getWorldPosition(event);
      if (!worldPos) return;
      const mapManager = mapManagerRef.current;
      const existingPoints = mapManager.getTopologyPoints();
      let pointIndex = existingPoints.length;
      let pointName = `NAV_POINT_${pointIndex}`;
      while (existingPoints.some(p => p.name === pointName)) {
        pointIndex++;
        pointName = `NAV_POINT_${pointIndex}`;
      }
      const newPoint: TopoPoint = {
        name: pointName,
        x: worldPos.x,
        y: worldPos.y,
        theta: 0,
        type: 0,
      };
      const command = new AddPointCommand(mapManager, newPoint, updateTopoMap);
      commandManagerRef.current.executeCommand(command);
      setSelectedPoint(newPoint);
      const topoLayer = layerManagerRef.current?.getLayer('topology');
      if (topoLayer instanceof TopoLayer) {
        topoLayer.setSelectedPoint(newPoint);
      }
      isPlacingPointRef.current = true;
      placingPointStartRef.current = worldPos.clone();
      placingPointInitialRef.current = { ...newPoint };
      createPlacingArrow(worldPos, worldPos);
      return;
    }

    if ((currentTool === 'brush' || currentTool === 'eraser') && event.button === 0) {
      // Prevent OrbitControls from intercepting
      event.preventDefault();
      event.stopPropagation();
      // Disable controls immediately
      if (controlsRef.current) {
        controlsRef.current.enablePan = false;
        controlsRef.current.enableRotate = false;
      }
      const worldPos = getWorldPosition(event);
      if (worldPos && occupancyGridLayerRef.current) {
        isDrawingRef.current = true;
        lastDrawPosRef.current = worldPos;
        currentGridChangesRef.current.clear();
        initialGridValuesRef.current.clear();

        const value = currentTool === 'brush' ? 100 : 0;
        const changes = occupancyGridLayerRef.current.modifyCells([{ x: worldPos.x, y: worldPos.y }], value, brushSize);

        for (const change of changes) {
          initialGridValuesRef.current.set(change.index, change.oldValue);
          currentGridChangesRef.current.set(change.index, { oldValue: change.oldValue, newValue: change.newValue });
        }


      }
      return;
    }

    if ((currentTool === 'rect' || currentTool === 'circle') && event.button === 0) {
      event.preventDefault();
      event.stopPropagation();
      if (controlsRef.current) {
        controlsRef.current.enablePan = false;
        controlsRef.current.enableRotate = false;
      }
      const worldPos = getWorldPosition(event);
      if (worldPos && occupancyGridLayerRef.current) {
        shapeStartRef.current = worldPos.clone();
        shapeEndRef.current = worldPos.clone();
        isPlacingShapeRef.current = true;
        createShapePreview(worldPos.x, worldPos.y, worldPos.x, worldPos.y);
      }
      return;
    }

    if (currentTool === 'move') {
      if (event.button === 0) {
        // 左键：移动点位
        const rect = canvasRef.current!.getBoundingClientRect();
        const mouse = new THREE.Vector2();
        mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

        raycasterRef.current!.setFromCamera(mouse, cameraRef.current!);
        const intersects = raycasterRef.current!.intersectObjects(sceneRef.current!.children, true);

        for (const intersect of intersects) {
          let obj = intersect.object;
          while (obj) {
            if (obj.userData.isTopoPoint && obj.userData.topoPoint) {
              const point = obj.userData.topoPoint;
              const mapManager = mapManagerRef.current;
              const pointData = mapManager.getTopologyPoint(point.name);
              if (pointData) {
                event.preventDefault();
                event.stopPropagation();
                setSelectedPoint(pointData);
                setIsDragging(true);
                setDragStartPos(new THREE.Vector2(event.clientX, event.clientY));
                dragStartPointRef.current = { ...pointData };

                // 禁用 controls
                if (controlsRef.current) {
                  controlsRef.current.enablePan = false;
                }

                // 找到对应的 group
                sceneRef.current!.traverse((child) => {
                  if (child instanceof THREE.Group && child.name === point.name) {
                    selectedPointRef.current = child;
                  }
                });
              }
              return;
            }
            obj = obj.parent as THREE.Object3D;
          }
        }
      } else if (event.button === 2) {
        // 右键：旋转点位方向
        const rect = canvasRef.current!.getBoundingClientRect();
        const mouse = new THREE.Vector2();
        mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

        raycasterRef.current!.setFromCamera(mouse, cameraRef.current!);
        const intersects = raycasterRef.current!.intersectObjects(sceneRef.current!.children, true);

        for (const intersect of intersects) {
          let obj = intersect.object;
          while (obj) {
            if (obj.userData.isTopoPoint && obj.userData.topoPoint) {
              const point = obj.userData.topoPoint;
              const mapManager = mapManagerRef.current;
              const pointData = mapManager.getTopologyPoint(point.name);
              if (pointData) {
                event.preventDefault();
                event.stopPropagation();
                setSelectedPoint(pointData);
                setIsRotating(true);
                setDragStartPos(new THREE.Vector2(event.clientX, event.clientY));
                dragStartPointRef.current = { ...pointData };

                // 禁用 controls
                if (controlsRef.current) {
                  controlsRef.current.enablePan = false;
                }

                // 找到对应的 group
                sceneRef.current!.traverse((child) => {
                  if (child instanceof THREE.Group && child.name === point.name) {
                    selectedPointRef.current = child;
                  }
                });
              }
              return;
            }
            obj = obj.parent as THREE.Object3D;
          }
        }
      }
    }

    // 旋转视角工具：左键拖拽旋转 yaw
    if (currentTool === 'rotateView' && event.button === 0) {
      event.preventDefault();
      event.stopPropagation();
      if (controlsRef.current) {
        controlsRef.current.enablePan = false;
      }
      isYawRotatingRef.current = true;
      yawStartMouseRef.current = { x: event.clientX, y: event.clientY, yaw: yawAngleRef.current };
      return;
    }
  };

  const updateBrushIndicatorFromNative = (event: MouseEvent) => {
    if ((currentTool === 'eraser' || currentTool === 'brush') && canvasRef.current && cameraRef.current) {
      const rect = canvasRef.current.getBoundingClientRect();
      setMousePosition({
        x: event.clientX - rect.left,
        y: event.clientY - rect.top,
      });
    } else {
      setMousePosition(null);
    }
  };

  const updateBrushIndicator = (event: React.MouseEvent<HTMLCanvasElement>) => {
    updateBrushIndicatorFromNative(event.nativeEvent);
  };

  const getBrushIndicatorSize = (): number => {
    if (!cameraRef.current || !canvasRef.current) return 0;

    const camera = cameraRef.current;
    const canvas = canvasRef.current;
    const distance = camera.position.z;
    const fov = camera.fov * (Math.PI / 180);
    const canvasHeight = canvas.clientHeight;
    const worldHeight = 2 * Math.tan(fov / 2) * distance;
    const pixelsPerMeter = canvasHeight / worldHeight;

    return brushSize * pixelsPerMeter;
  };

  const handleCanvasMouseMove = (event: MouseEvent) => {
    updateBrushIndicatorFromNative(event);

    const worldPos = getWorldPosition(event);
    if (worldPos) {
      setMouseWorldPos({ x: worldPos.x, y: worldPos.y });
    }

    // 添加点位拖拽：更新点位方向
    if (isPlacingPointRef.current && placingPointStartRef.current && placingPointInitialRef.current) {
      if (!worldPos) return;
      event.preventDefault();
      event.stopPropagation();
      const dx = worldPos.x - placingPointStartRef.current.x;
      const dy = worldPos.y - placingPointStartRef.current.y;
      const theta = Math.atan2(dy, dx);
      const currentPoint = mapManagerRef.current.getTopologyPoint(placingPointInitialRef.current.name);
      if (currentPoint) {
        const updatedPoint: TopoPoint = { ...currentPoint, theta };
        mapManagerRef.current.setTopologyPoint(updatedPoint);
        setSelectedPoint(updatedPoint);
        updateTopoMap();
      }
      updatePlacingArrow(placingPointStartRef.current, worldPos);
      return;
    }

    if (isDrawingRef.current && (currentTool === 'brush' || currentTool === 'eraser') && occupancyGridLayerRef.current) {
      if (!worldPos) return;
      // Must have left button held (touch 事件 event.buttons 恒为 0，用 isTouchSessionRef 绕过)
      if (!isTouchSessionRef.current && !(event.buttons & 1)) {
        isDrawingRef.current = false;
        lastDrawPosRef.current = null;
        return;
      }
      event.preventDefault();
      event.stopPropagation();

      // Throttle to ~30fps using requestAnimationFrame
      if (drawRafIdRef.current !== null) return;
      const evtButtons = event.buttons; // capture for RAF callback
      drawRafIdRef.current = requestAnimationFrame(() => {
        drawRafIdRef.current = null;
        if (worldPos) {
        const value = currentTool === 'brush' ? 100 : 0;
        const positions: Array<{ x: number; y: number }> = [];

        if (lastDrawPosRef.current) {
          const dx = worldPos.x - lastDrawPosRef.current.x;
          const dy = worldPos.y - lastDrawPosRef.current.y;
          const dist = Math.sqrt(dx * dx + dy * dy);
          const steps = Math.max(1, Math.ceil(dist / (brushSize / 2)));

          for (let i = 0; i <= steps; i++) {
            const t = i / steps;
            positions.push({
              x: lastDrawPosRef.current.x + dx * t,
              y: lastDrawPosRef.current.y + dy * t,
            });
          }
        } else {
          positions.push({ x: worldPos.x, y: worldPos.y });
        }

        const changes = occupancyGridLayerRef.current.modifyCells(positions, value, brushSize, initialGridValuesRef.current);

        for (const change of changes) {
          if (!currentGridChangesRef.current.has(change.index)) {
            if (!initialGridValuesRef.current.has(change.index)) {
              initialGridValuesRef.current.set(change.index, change.oldValue);
            }
            currentGridChangesRef.current.set(change.index, { oldValue: change.oldValue, newValue: change.newValue });
          } else {
            const existing = currentGridChangesRef.current.get(change.index)!;
            existing.newValue = change.newValue;
          }
        }

        // Flush to undo every 5000 cells to avoid Map growing too large (scans_4 has 42M cells)
        if (currentGridChangesRef.current.size > 5000) {
          const arr: GridCellChange[] = [];
          for (const [index, values] of currentGridChangesRef.current) {
            arr.push({ index, oldValue: values.oldValue, newValue: values.newValue });
          }
          commandManagerRef.current.executeCommand(new ModifyGridCommand(occupancyGridLayerRef.current!, arr, () => {}));
          currentGridChangesRef.current.clear();
        }

        lastDrawPosRef.current = worldPos;
      }
      });
      return;
    }

    if (isRotating && selectedPoint && dragStartPos && selectedPointRef.current) {
      // 右键拖动：旋转点位方向
      event.preventDefault();
      event.stopPropagation();

      const worldPos = getWorldPosition(event);
      if (worldPos && selectedPointRef.current) {
        const dx = worldPos.x - selectedPoint.x;
        const dy = worldPos.y - selectedPoint.y;
        const theta = -Math.atan2(dy, dx);
        const updatedPoint: TopoPoint = {
          ...selectedPoint,
          theta: theta,
        };
        mapManagerRef.current.setTopologyPoint(updatedPoint);
        setSelectedPoint(updatedPoint);
        updateTopoMap();
      }
    } else if (isDragging && selectedPoint && dragStartPos && selectedPointRef.current) {
      event.preventDefault();
      event.stopPropagation();

      // 左键拖动：移动位置
      const worldPos = getWorldPosition(event);
      if (worldPos) {
        const updatedPoint: TopoPoint = {
          ...selectedPoint,
          x: worldPos.x,
          y: worldPos.y,
        };
        mapManagerRef.current.setTopologyPoint(updatedPoint);
        setSelectedPoint(updatedPoint);
        updateTopoMap();
      }
    } else if (currentTool === 'drawLine' && lineStartPoint) {
      // 更新预览线段（直线绘制）
      updatePreviewLine(event);
    } else if (isPlacingShapeRef.current && (currentTool === 'rect' || currentTool === 'circle')) {
      if (!(event.buttons & 1)) {
        isPlacingShapeRef.current = false;
        clearShapePreview();
        return;
      }
      event.preventDefault();
      event.stopPropagation();
      updateShapePreview(event);
    }

    // yaw 旋转视角
    if (isYawRotatingRef.current && yawStartMouseRef.current) {
      const dx = event.clientX - yawStartMouseRef.current.x;
      const sensitivity = 0.01; // 弧度/像素
      yawAngleRef.current = yawStartMouseRef.current.yaw + dx * sensitivity;
    }
  };

  const handleCanvasMouseUp = () => {
    // 结束添加点位拖拽
    if (isPlacingPointRef.current && placingPointInitialRef.current) {
      const finalPoint = mapManagerRef.current.getTopologyPoint(placingPointInitialRef.current.name);
      if (finalPoint && finalPoint.theta !== 0) {
        // 创建 modify command 使方向变化可撤销
        const modifyCmd = new ModifyPointCommand(
          mapManagerRef.current,
          placingPointInitialRef.current,
          finalPoint,
          updateTopoMap,
        );
        commandManagerRef.current.executeCommand(modifyCmd);
      }
      clearPlacingArrow();
      isPlacingPointRef.current = false;
      placingPointStartRef.current = null;
      const pointName = placingPointInitialRef.current.name;
      placingPointInitialRef.current = null;
      if (controlsRef.current) {
        controlsRef.current.enablePan = true;
      }
      toast.success(`已添加点位: ${pointName}`);
    }

    if (isDrawingRef.current && (currentTool === 'brush' || currentTool === 'eraser')) {
      if (currentGridChangesRef.current.size > 0) {
        const changes: GridCellChange[] = [];
        for (const [index, values] of currentGridChangesRef.current) {
          changes.push({ index, oldValue: values.oldValue, newValue: values.newValue });
        }
        if (changes.length > 0) {
          commandManagerRef.current.executeCommand(new ModifyGridCommand(occupancyGridLayerRef.current!, changes, () => {}));
        }
        currentGridChangesRef.current.clear();
      }
      isDrawingRef.current = false;
      if (drawRafIdRef.current !== null) {
        cancelAnimationFrame(drawRafIdRef.current);
        drawRafIdRef.current = null;
      }
      lastDrawPosRef.current = null;
    }

    if ((isDragging || isRotating) && selectedPoint && dragStartPointRef.current) {
      const currentPoint = mapManagerRef.current.getTopologyPoint(selectedPoint.name);
      if (currentPoint && dragStartPointRef.current.name === currentPoint.name) {
        const command = new ModifyPointCommand(mapManagerRef.current, dragStartPointRef.current, currentPoint, updateTopoMap);
        commandManagerRef.current.executeCommand(command);
      }
      dragStartPointRef.current = null;
    }

    if (isPlacingShapeRef.current && shapeStartRef.current && shapeEndRef.current && (currentTool === 'rect' || currentTool === 'circle')) {
      const layer = occupancyGridLayerRef.current;
      if (layer) {
        const start = shapeStartRef.current;
        const end = shapeEndRef.current;
        const changes: GridCellChange[] = [];
        const value = 100;

        if (currentTool === 'rect') {
          changes.push(...layer.drawRect(start.x, start.y, end.x, end.y, value));
        } else if (currentTool === 'circle') {
          const dx = end.x - start.x;
          const dy = end.y - start.y;
          const r = Math.sqrt(dx * dx + dy * dy);
          changes.push(...layer.drawCircle(start.x, start.y, r, value));
        }

        if (changes.length > 0) {
          const command = new ModifyGridCommand(layer, changes, () => {});
          commandManagerRef.current.executeCommand(command);
        }
      }
      clearShapePreview();
      shapeStartRef.current = null;
      shapeEndRef.current = null;
      isPlacingShapeRef.current = false;
    }

    setIsDragging(false);
    setIsRotating(false);
    setDragStartPos(null);
    selectedPointRef.current = null;

    // 结束 yaw 旋转
    isYawRotatingRef.current = false;
    yawStartMouseRef.current = null;

    // 恢复 OrbitControls，确保移动工具在绘制/拖拽后仍可平移
    if (controlsRef.current) {
      controlsRef.current.enablePan = true;
      controlsRef.current.enableRotate = false; // 始终保持2D模式
    }
  };

  const createPreviewLine = (startX: number, startY: number, endX: number, endY: number) => {
    if (!sceneRef.current) return;

    clearPreviewLine();

    const geometry = new THREE.BufferGeometry();
    const pointHeight = 0.2 * 2;
    const lineZ = 0.002 + pointHeight / 2;
    const positions = new Float32Array([
      startX,
      startY,
      lineZ,
      endX,
      endY,
      lineZ,
    ]);
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    const material = new THREE.LineBasicMaterial({
      color: 0x2196f3,
      linewidth: 2,
      transparent: true,
      opacity: 0.6,
    });

    const line = new THREE.Line(geometry, material);
    line.name = 'previewLine';
    previewLineRef.current = line;
    sceneRef.current.add(line);
  };

  const updatePreviewLine = (event: MouseEvent | { clientX: number; clientY: number }) => {
    if (!sceneRef.current) return;

    if (currentTool === 'drawLine' && lineStartPoint) {
      const worldPos = getWorldPosition(event);
      if (!worldPos) return;

      if (!previewLineRef.current) {
        createPreviewLine(lineStartPoint.x, lineStartPoint.y, worldPos.x, worldPos.y);
        return;
      }

      const geometry = previewLineRef.current.geometry as THREE.BufferGeometry;
      const positions = geometry.attributes.position.array as Float32Array;
      const pointHeight = 0.2 * 2;
      const lineZ = 0.002 + pointHeight / 2;

      positions[0] = lineStartPoint.x;
      positions[1] = lineStartPoint.y;
      positions[2] = lineZ;
      positions[3] = worldPos.x;
      positions[4] = worldPos.y;
      positions[5] = lineZ;

      geometry.attributes.position.needsUpdate = true;
    }
  };

  const clearPreviewLine = () => {
    if (previewLineRef.current && sceneRef.current) {
      sceneRef.current.remove(previewLineRef.current);
      previewLineRef.current.geometry.dispose();
      (previewLineRef.current.material as THREE.Material).dispose();
      previewLineRef.current = null;
    }
  };

  const createPlacingArrow = (start: THREE.Vector3, end: THREE.Vector3) => {
    if (!sceneRef.current) return;
    clearPlacingArrow();
    const geometry = new THREE.BufferGeometry().setFromPoints([start.clone(), end.clone()]);
    const material = new THREE.LineBasicMaterial({
      color: 0xff4444,
      depthTest: false,
      transparent: true,
    });
    placingPointArrowRef.current = new THREE.Line(geometry, material);
    placingPointArrowRef.current.renderOrder = 10000;
    sceneRef.current.add(placingPointArrowRef.current);
  };

  const updatePlacingArrow = (start: THREE.Vector3, end: THREE.Vector3) => {
    if (!placingPointArrowRef.current || !sceneRef.current) return;
    sceneRef.current.remove(placingPointArrowRef.current);
    placingPointArrowRef.current.geometry?.dispose();
    (placingPointArrowRef.current.material as THREE.Material)?.dispose();
    const geometry = new THREE.BufferGeometry().setFromPoints([start.clone(), end.clone()]);
    const material = new THREE.LineBasicMaterial({
      color: 0xff4444,
      depthTest: false,
      transparent: true,
    });
    placingPointArrowRef.current = new THREE.Line(geometry, material);
    placingPointArrowRef.current.renderOrder = 10000;
    sceneRef.current.add(placingPointArrowRef.current);
  };

  const clearPlacingArrow = () => {
    if (!placingPointArrowRef.current || !sceneRef.current) return;
    sceneRef.current.remove(placingPointArrowRef.current);
    placingPointArrowRef.current.geometry?.dispose();
    (placingPointArrowRef.current.material as THREE.Material)?.dispose();
    placingPointArrowRef.current = null;
  };

  const clearShapePreview = () => {
    if (shapePreviewRef.current && sceneRef.current) {
      sceneRef.current.remove(shapePreviewRef.current);
      shapePreviewRef.current.geometry.dispose();
      (shapePreviewRef.current.material as THREE.Material).dispose();
      shapePreviewRef.current = null;
    }
    if (shapeBorderRef.current && sceneRef.current) {
      sceneRef.current.remove(shapeBorderRef.current);
      shapeBorderRef.current.geometry.dispose();
      (shapeBorderRef.current.material as THREE.Material).dispose();
      shapeBorderRef.current = null;
    }
  };

  const createShapePreview = (x1: number, y1: number, x2: number, y2: number) => {
    if (!sceneRef.current) return;
    clearShapePreview();

    const z = 0.003; // Slightly above grid
    const color = 0xff4444;

    if (currentTool === 'rect') {
      const cx = (x1 + x2) / 2;
      const cy = (y1 + y2) / 2;
      const w = Math.abs(x2 - x1);
      const h = Math.abs(y2 - y1);

      if (w > 0 && h > 0) {
        const geometry = new THREE.PlaneGeometry(w, h);
        const material = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.3, side: THREE.DoubleSide });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(cx, cy, z);
        mesh.name = 'shapePreview';
        shapePreviewRef.current = mesh;
        sceneRef.current.add(mesh);

        // Border
        const borderGeom = new THREE.BufferGeometry();
        const hw = w / 2, hh = h / 2;
        const verts = new Float32Array([
          -hw, -hh, z + 0.001,
          hw, -hh, z + 0.001,
          hw, hh, z + 0.001,
          -hw, hh, z + 0.001,
          -hw, -hh, z + 0.001,
        ]);
        borderGeom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
        const borderMat = new THREE.LineBasicMaterial({ color, linewidth: 2 });
        const border = new THREE.Line(borderGeom, borderMat);
        border.position.set(cx, cy, 0);
        border.name = 'shapeBorder';
        shapeBorderRef.current = border;
        sceneRef.current.add(border);
      }
    } else if (currentTool === 'circle') {
      const dx = x2 - x1;
      const dy = y2 - y1;
      const r = Math.sqrt(dx * dx + dy * dy);

      if (r > 0.01) {
        const geometry = new THREE.CircleGeometry(r, 64);
        const material = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.3, side: THREE.DoubleSide });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(x1, y1, z);
        mesh.name = 'shapePreview';
        shapePreviewRef.current = mesh;
        sceneRef.current.add(mesh);

        // Border ring
        const ringGeom = new THREE.BufferGeometry();
        const points: number[] = [];
        const segments = 64;
        for (let i = 0; i <= segments; i++) {
          const angle = (i / segments) * Math.PI * 2;
          points.push(Math.cos(angle) * r, Math.sin(angle) * r, z + 0.001);
        }
        ringGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(points), 3));
        const ringMat = new THREE.LineBasicMaterial({ color, linewidth: 2 });
        const ring = new THREE.Line(ringGeom, ringMat);
        ring.position.set(x1, y1, 0);
        ring.name = 'shapeBorder';
        shapeBorderRef.current = ring;
        sceneRef.current.add(ring);
      }
    }
  };

  const updateShapePreview = (event: MouseEvent) => {
    if (!isPlacingShapeRef.current || !shapeStartRef.current) return;
    const worldPos = getWorldPosition(event);
    if (!worldPos) return;
    shapeEndRef.current = worldPos.clone();
    createShapePreview(shapeStartRef.current.x, shapeStartRef.current.y, worldPos.x, worldPos.y);
  };

  const updateTopoMap = () => {
    const mapManager = mapManagerRef.current;
    const topologyMap = mapManager.getTopologyMap();

    const mapProperty = mapManager.getMapProperty();
    const defaultControllers = ['FollowPath'];
    const defaultGoalCheckers = ['general_goal_checker'];

    const allControllers = new Set<string>(defaultControllers);
    const allGoalCheckers = new Set<string>(defaultGoalCheckers);

    if (mapProperty) {
      if (mapProperty.support_controllers) {
        mapProperty.support_controllers.forEach(c => allControllers.add(c));
      }
      if (mapProperty.support_goal_checkers) {
        mapProperty.support_goal_checkers.forEach(g => allGoalCheckers.add(g));
      }
    }

    const routes = mapManager.getTopologyRoutes();
    routes.forEach(route => {
      if (route.route_info.controller) {
        allControllers.add(route.route_info.controller);
      }
      if (route.route_info.goal_checker) {
        allGoalCheckers.add(route.route_info.goal_checker);
      }
    });

    setSupportControllers(Array.from(allControllers).sort());
    setSupportGoalCheckers(Array.from(allGoalCheckers).sort());

    if (topoLayerRef.current) {
      topoLayerRef.current.update(topologyMap);
      // 更新后恢复选中状态
      const currentSelectedPoint = selectedPointStateRef.current;
      if (currentSelectedPoint) {
        const currentPoint = mapManager.getTopologyPoint(currentSelectedPoint.name);
        if (currentPoint) {
          topoLayerRef.current.setSelectedPoint(currentPoint);
        }
      }
    }
  };

  const handleSave = () => {
    const mapManager = mapManagerRef.current;

    const defaultControllers = ['FollowPath'];
    const defaultGoalCheckers = ['general_goal_checker'];

    const allControllers = new Set<string>(defaultControllers);
    const allGoalCheckers = new Set<string>(defaultGoalCheckers);

    const existingMapProperty = mapManager.getMapProperty();
    if (existingMapProperty) {
      if (existingMapProperty.support_controllers) {
        existingMapProperty.support_controllers.forEach(c => allControllers.add(c));
      }
      if (existingMapProperty.support_goal_checkers) {
        existingMapProperty.support_goal_checkers.forEach(g => allGoalCheckers.add(g));
      }
    }

    const routes = mapManager.getTopologyRoutes();
    routes.forEach(route => {
      if (route.route_info.controller) {
        allControllers.add(route.route_info.controller);
      }
      if (route.route_info.goal_checker) {
        allGoalCheckers.add(route.route_info.goal_checker);
      }
    });

    const mapProperty = {
      support_controllers: Array.from(allControllers).sort(),
      support_goal_checkers: Array.from(allGoalCheckers).sort(),
    };

    mapManager.updateMapProperty(mapProperty);

    try {
      mapManager.saveAndPublishTopology(connection);
      toast.success('拓扑地图已保存并发布');
    } catch (error) {
      console.error('Failed to save/publish topology map:', error);
      mapManager.saveTopology();
      toast.warning('保存成功，但发布失败');
    }

    try {
      mapManagerRef.current.publishOccupancyGrid(connection);
      toast.success('栅格地图已发布到 /map/update');
    } catch (error) {
      console.error('Failed to publish occupancy grid:', error);
      toast.warning('栅格地图发布失败');
    }
  };

  useEffect(() => {
    selectedPointStateRef.current = selectedPoint;
  }, [selectedPoint]);

  useEffect(() => {
    if (selectedPoint) {
      setEditingPoint({ ...selectedPoint });
    } else {
      setEditingPoint(null);
    }
  }, [selectedPoint]);

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if ((event.ctrlKey || event.metaKey) && event.key === 'z') {
        event.preventDefault();
        if (event.shiftKey) {
          if (commandManagerRef.current.redo()) {
            toast.success('重做');
          }
        } else {
          if (commandManagerRef.current.undo()) {
            toast.success('撤销');
          }
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const clickHandler = (e: MouseEvent) => handleCanvasClick(e);
    const mouseDownHandler = (e: MouseEvent) => handleCanvasMouseDown(e);
    const mouseMoveHandler = (e: MouseEvent) => handleCanvasMouseMove(e);
    const mouseUpHandler = () => handleCanvasMouseUp();
    const mouseLeaveHandler = () => setMouseWorldPos(null);

    // ───── 触摸事件（平板）：分流工具操作与地图平移 ─────
    // 核心原则：单指走工具逻辑，双指始终走 OrbitControls 平移/旋转/缩放
    const touchStartHandler = (e: TouchEvent) => {
      if (e.touches.length >= 2) {
        // 双指及以上：取消所有工具操作，恢复 OrbitControls 平移/旋转
        // 1) 结束画笔绘制
        if (isDrawingRef.current) {
          if (currentGridChangesRef.current.size > 0) {
            const changes: GridCellChange[] = (() => { const arr: GridCellChange[] = []; for (const [index, values] of currentGridChangesRef.current) { arr.push({ index, oldValue: values.oldValue, newValue: values.newValue }); } return arr; })();
            if (occupancyGridLayerRef.current) {
              const cmd = new ModifyGridCommand(occupancyGridLayerRef.current, changes, () => {});
              commandManagerRef.current.executeCommand(cmd);
            }
            currentGridChangesRef.current.clear();
          }
          isDrawingRef.current = false;
          if (drawRafIdRef.current !== null) {
            cancelAnimationFrame(drawRafIdRef.current);
            drawRafIdRef.current = null;
          }
          lastDrawPosRef.current = null;
        }
        // 2) 结束形状放置
        if (isPlacingShapeRef.current) {
          clearShapePreview();
          isPlacingShapeRef.current = false;
          shapeStartRef.current = null;
          shapeEndRef.current = null;
        }
        // 3) 结束点位拖拽/旋转
        if (isDragging || isRotating) {
          if (selectedPoint && dragStartPointRef.current) {
            const currentPoint = mapManagerRef.current.getTopologyPoint(selectedPoint.name);
            if (currentPoint && dragStartPointRef.current.name === currentPoint.name) {
              const cmd = new ModifyPointCommand(mapManagerRef.current, dragStartPointRef.current, currentPoint, updateTopoMap);
              commandManagerRef.current.executeCommand(cmd);
            }
            dragStartPointRef.current = null;
          }
          setIsDragging(false);
          setIsRotating(false);
          setDragStartPos(null);
          selectedPointRef.current = null;
        }
        // 4) 结束视角旋转
        if (isYawRotatingRef.current) {
          isYawRotatingRef.current = false;
          yawStartMouseRef.current = null;
        }
        // 5) 结束点位放置拖拽
        if (isPlacingPointRef.current) {
          clearPlacingArrow();
          isPlacingPointRef.current = false;
          placingPointStartRef.current = null;
          placingPointInitialRef.current = null;
        }
        // 恢复 OrbitControls 平移和旋转，让它处理双指手势
        if (controlsRef.current) {
          controlsRef.current.enablePan = true;
          controlsRef.current.enableRotate = true;
        }
        // 不 preventDefault，让 OrbitControls 原生接收触摸事件
        return;
      }

      // 单指触摸：标记触摸会话，避免 mousemove 的 event.buttons 检查误杀
      isTouchSessionRef.current = true;

      // ── 旋转视角工具 ──
      if (currentTool === 'rotateView') {
        e.preventDefault();
        const touch = e.touches[0]!;
        if (controlsRef.current) {
          controlsRef.current.enablePan = false;
          controlsRef.current.enableRotate = false;
        }
        isYawRotatingRef.current = true;
        yawStartMouseRef.current = { x: touch.clientX, y: touch.clientY, yaw: yawAngleRef.current };
        return;
      }

      // ── 画笔 / 橡皮擦：需提前禁用 pan，防止 OrbitControls pointerdown 抢先 ──
      if (currentTool === 'brush' || currentTool === 'eraser') {
        e.preventDefault();
        const touch = e.touches[0]!;
        if (controlsRef.current) {
          controlsRef.current.enablePan = false;
          controlsRef.current.enableRotate = false;
        }
        const worldPos = getWorldPosition({ clientX: touch.clientX, clientY: touch.clientY });
        if (worldPos && occupancyGridLayerRef.current) {
          isDrawingRef.current = true;
          lastDrawPosRef.current = worldPos;
          currentGridChangesRef.current.clear();
          initialGridValuesRef.current.clear();
          const value = currentTool === 'brush' ? 100 : 0;
          const changes = occupancyGridLayerRef.current.modifyCells(
            [{ x: worldPos.x, y: worldPos.y }], value, brushSize,
          );
          for (const change of changes) {
            initialGridValuesRef.current.set(change.index, change.oldValue);
            currentGridChangesRef.current.set(change.index, { oldValue: change.oldValue, newValue: change.newValue });
          }
        }
        return;
      }

      // ── 矩形 / 圆形：提前禁用 pan ──
      if (currentTool === 'rect' || currentTool === 'circle') {
        e.preventDefault();
        const touch = e.touches[0]!;
        if (controlsRef.current) {
          controlsRef.current.enablePan = false;
          controlsRef.current.enableRotate = false;
        }
        const worldPos = getWorldPosition({ clientX: touch.clientX, clientY: touch.clientY });
        if (worldPos && occupancyGridLayerRef.current) {
          shapeStartRef.current = worldPos.clone();
          shapeEndRef.current = worldPos.clone();
          isPlacingShapeRef.current = true;
          createShapePreview(worldPos.x, worldPos.y, worldPos.x, worldPos.y);
        }
        return;
      }

      // ── 直线绘制：提前禁用 pan ──
      if (currentTool === 'drawLine') {
        e.preventDefault();
        const touch = e.touches[0]!;
        if (controlsRef.current) {
          controlsRef.current.enablePan = false;
          controlsRef.current.enableRotate = false;
        }
        const worldPos = getWorldPosition({ clientX: touch.clientX, clientY: touch.clientY });
        if (!worldPos) return;
        if (!lineStartPoint) {
          setLineStartPoint(worldPos);
          createPreviewLine(worldPos.x, worldPos.y, worldPos.x, worldPos.y);
          toast.info('已设置起始点，点击设置结束点');
        } else {
          const endPos = worldPos;
          const changes = occupancyGridLayerRef.current!.drawLine(
            lineStartPoint.x, lineStartPoint.y, endPos.x, endPos.y, 100, brushSize,
          );
          if (changes.length > 0) {
            const command = new ModifyGridCommand(occupancyGridLayerRef.current!, changes, () => {});
            commandManagerRef.current.executeCommand(command);
          }
          clearPreviewLine();
          setLineStartPoint(null);
          toast.success('已绘制直线');
        }
        return;
      }

      // ── 添加点位：preventDefault 防止 OrbitControls 接管 + 直接创建点位 ──
      if (currentTool === 'addPoint') {
        e.preventDefault();
        const touch = e.touches[0]!;
        if (controlsRef.current) {
          controlsRef.current.enablePan = false;
          controlsRef.current.enableRotate = false;
        }
        const worldPos = getWorldPosition({ clientX: touch.clientX, clientY: touch.clientY });
        if (!worldPos) return;

        // 先检查是否点中已有点位
        const mouse = new THREE.Vector2(
          ((touch.clientX - canvasRef.current!.getBoundingClientRect().left) / canvasRef.current!.getBoundingClientRect().width) * 2 - 1,
          -((touch.clientY - canvasRef.current!.getBoundingClientRect().top) / canvasRef.current!.getBoundingClientRect().height) * 2 + 1,
        );
        raycasterRef.current!.setFromCamera(mouse, cameraRef.current!);
        const intersects = raycasterRef.current!.intersectObjects(sceneRef.current!.children, true);
        for (const intersect of intersects) {
          let obj = intersect.object;
          while (obj) {
            if (obj.userData.isTopoPoint) {
              // 已有点位：选中而不创建
              return;
            }
            obj = obj.parent as THREE.Object3D;
          }
        }

        const mapManager = mapManagerRef.current;
        const existingPoints = mapManager.getTopologyPoints();
        let pointIndex = existingPoints.length;
        let pointName = `NAV_POINT_${pointIndex}`;
        while (existingPoints.some(p => p.name === pointName)) {
          pointIndex++;
          pointName = `NAV_POINT_${pointIndex}`;
        }
        const newPoint: TopoPoint = {
          name: pointName,
          x: worldPos.x,
          y: worldPos.y,
          theta: 0,
          type: 0,
        };
        const command = new AddPointCommand(mapManager, newPoint, updateTopoMap);
        commandManagerRef.current.executeCommand(command);
        setSelectedPoint(newPoint);
        const topoLayer = layerManagerRef.current?.getLayer('topology');
        if (topoLayer instanceof TopoLayer) {
          topoLayer.setSelectedPoint(newPoint);
        }
        isPlacingPointRef.current = true;
        placingPointStartRef.current = worldPos.clone();
        placingPointInitialRef.current = { ...newPoint };
        createPlacingArrow(worldPos, worldPos);
        return;
      }

      // ── 移动 / 点击添加直线终点：保持 pan 启用，交由 mousedown 处理 ──
      // 不 preventDefault，浏览器会生成 mousedown → handleCanvasMouseDown 接管
    };

    const touchMoveHandler = (e: TouchEvent) => {
      // 双指以上：确保 OrbitControls 保持启用
      if (e.touches.length >= 2) {
        if (controlsRef.current) {
          controlsRef.current.enablePan = true;
          controlsRef.current.enableRotate = true;
        }
        return;
      }

      const touch = e.touches[0]!;
      const worldPos = getWorldPosition({ clientX: touch.clientX, clientY: touch.clientY });
      if (worldPos) {
        setMouseWorldPos({ x: worldPos.x, y: worldPos.y });
      }

      // 单指点位放置拖拽：阻止页面滚动 + 更新方向
      if (isPlacingPointRef.current && placingPointStartRef.current && placingPointInitialRef.current) {
        e.preventDefault();
        if (worldPos) {
          const dx = worldPos.x - placingPointStartRef.current.x;
          const dy = worldPos.y - placingPointStartRef.current.y;
          const theta = Math.atan2(dy, dx);
          const currentPoint = mapManagerRef.current.getTopologyPoint(placingPointInitialRef.current.name);
          if (currentPoint) {
            const updatedPoint: TopoPoint = { ...currentPoint, theta };
            mapManagerRef.current.setTopologyPoint(updatedPoint);
            setSelectedPoint(updatedPoint);
            updateTopoMap();
          }
          updatePlacingArrow(placingPointStartRef.current, worldPos);
        }
        return;
      }

      // 单指旋转视角
      if (isYawRotatingRef.current && yawStartMouseRef.current) {
        e.preventDefault();
        const dx = touch.clientX - yawStartMouseRef.current.x;
        const sensitivity = 0.01;
        yawAngleRef.current = yawStartMouseRef.current.yaw + dx * sensitivity;
        return;
      }

      // 单指画笔/橡皮擦滑动
      if (isDrawingRef.current && (currentTool === 'brush' || currentTool === 'eraser') && occupancyGridLayerRef.current) {
        e.preventDefault();
        if (!worldPos) return;
        // Throttle to ~30fps
        if (drawRafIdRef.current !== null) return;
        drawRafIdRef.current = requestAnimationFrame(() => {
          drawRafIdRef.current = null;
          const w = worldPos;
          if (w) {
            const value = currentTool === 'brush' ? 100 : 0;
            const positions: Array<{ x: number; y: number }> = [];
            if (lastDrawPosRef.current) {
              const dx = w.x - lastDrawPosRef.current.x;
              const dy = w.y - lastDrawPosRef.current.y;
              const dist = Math.sqrt(dx * dx + dy * dy);
              const steps = Math.max(1, Math.ceil(dist / (brushSize / 2)));
              for (let i = 0; i <= steps; i++) {
                const t = i / steps;
                positions.push({
                  x: lastDrawPosRef.current.x + dx * t,
                  y: lastDrawPosRef.current.y + dy * t,
                });
              }
            } else {
              positions.push({ x: w.x, y: w.y });
            }
            const changes = occupancyGridLayerRef.current!.modifyCells(positions, value, brushSize, initialGridValuesRef.current);
            for (const change of changes) {
              if (!currentGridChangesRef.current.has(change.index)) {
                if (!initialGridValuesRef.current.has(change.index)) {
                  initialGridValuesRef.current.set(change.index, change.oldValue);
                }
                currentGridChangesRef.current.set(change.index, { oldValue: change.oldValue, newValue: change.newValue });
              } else {
                const existing = currentGridChangesRef.current.get(change.index)!;
                existing.newValue = change.newValue;
              }
            }
            lastDrawPosRef.current = w;
          }
        });
        return;
      }

      // 单指矩形/圆形拖拽调整大小
      if (isPlacingShapeRef.current && (currentTool === 'rect' || currentTool === 'circle')) {
        e.preventDefault();
        if (worldPos && shapeStartRef.current) {
          shapeEndRef.current = worldPos.clone();
          createShapePreview(shapeStartRef.current.x, shapeStartRef.current.y, worldPos.x, worldPos.y);
        }
        return;
      }

      // 单指直线绘制预览
      if (currentTool === 'drawLine' && lineStartPoint) {
        e.preventDefault();
        updatePreviewLine({ clientX: touch.clientX, clientY: touch.clientY });
        return;
      }
    };

    const touchEndHandler = (e: TouchEvent) => {
      if (e.touches.length === 0) {
        // ── 所有手指离开：清理触摸会话标记 ──
        isTouchSessionRef.current = false;

        // 画笔/橡皮擦结束：提交修改
        if (isDrawingRef.current && (currentTool === 'brush' || currentTool === 'eraser')) {
          if (currentGridChangesRef.current.size > 0) {
            const changes: GridCellChange[] = (() => { const arr: GridCellChange[] = []; for (const [index, values] of currentGridChangesRef.current) { arr.push({ index, oldValue: values.oldValue, newValue: values.newValue }); } return arr; })();
            if (occupancyGridLayerRef.current) {
              const cmd = new ModifyGridCommand(occupancyGridLayerRef.current, changes, () => {});
              commandManagerRef.current.executeCommand(cmd);
            }
            currentGridChangesRef.current.clear();
          }
          isDrawingRef.current = false;
          if (drawRafIdRef.current !== null) {
            cancelAnimationFrame(drawRafIdRef.current);
            drawRafIdRef.current = null;
          }
          lastDrawPosRef.current = null;
        }

        // 矩形/圆形结束：提交形状
        if (isPlacingShapeRef.current && shapeStartRef.current && shapeEndRef.current && (currentTool === 'rect' || currentTool === 'circle')) {
          const layer = occupancyGridLayerRef.current;
          if (layer) {
            const start = shapeStartRef.current;
            const end = shapeEndRef.current;
            const changes: GridCellChange[] = [];
            const value = 100;
            if (currentTool === 'rect') {
              changes.push(...layer.drawRect(start.x, start.y, end.x, end.y, value));
            } else if (currentTool === 'circle') {
              const dx = end.x - start.x;
              const dy = end.y - start.y;
              const r = Math.sqrt(dx * dx + dy * dy);
              changes.push(...layer.drawCircle(start.x, start.y, r, value));
            }
            if (changes.length > 0) {
              const cmd = new ModifyGridCommand(layer, changes, () => {});
              commandManagerRef.current.executeCommand(cmd);
            }
          }
          clearShapePreview();
          shapeStartRef.current = null;
          shapeEndRef.current = null;
          isPlacingShapeRef.current = false;
        }

        // 旋转视角结束
        if (isYawRotatingRef.current) {
          isYawRotatingRef.current = false;
          yawStartMouseRef.current = null;
        }

        // 添加点位拖拽结束
        if (isPlacingPointRef.current && placingPointInitialRef.current) {
          const finalPoint = mapManagerRef.current.getTopologyPoint(placingPointInitialRef.current.name);
          if (finalPoint && finalPoint.theta !== 0) {
            const modifyCmd = new ModifyPointCommand(
              mapManagerRef.current,
              placingPointInitialRef.current,
              finalPoint,
              updateTopoMap,
            );
            commandManagerRef.current.executeCommand(modifyCmd);
          }
          clearPlacingArrow();
          const pointName = placingPointInitialRef.current.name;
          isPlacingPointRef.current = false;
          placingPointStartRef.current = null;
          placingPointInitialRef.current = null;
          toast.success(`已添加点位: ${pointName}`);
        }

        // 恢复 OrbitControls
        if (controlsRef.current) {
          controlsRef.current.enablePan = true;
          controlsRef.current.enableRotate = false; // 编辑器保持 2D
        }
      }
      // move / addPoint 工具未 preventDefault，mouseUp 会由浏览器自动生成处理
    };

    canvas.addEventListener('click', clickHandler);
    canvas.addEventListener('mousedown', mouseDownHandler);
    canvas.addEventListener('mousemove', mouseMoveHandler);
    canvas.addEventListener('mouseup', mouseUpHandler);
    canvas.addEventListener('mouseleave', mouseLeaveHandler);
    canvas.addEventListener('touchstart', touchStartHandler, { passive: false });
    canvas.addEventListener('touchmove', touchMoveHandler, { passive: false });
    canvas.addEventListener('touchend', touchEndHandler);
    canvas.addEventListener('touchcancel', touchEndHandler);

    return () => {
      canvas.removeEventListener('click', clickHandler);
      canvas.removeEventListener('mousedown', mouseDownHandler);
      canvas.removeEventListener('mousemove', mouseMoveHandler);
      canvas.removeEventListener('mouseup', mouseUpHandler);
      canvas.removeEventListener('mouseleave', mouseLeaveHandler);
      canvas.removeEventListener('touchstart', touchStartHandler);
      canvas.removeEventListener('touchmove', touchMoveHandler);
      canvas.removeEventListener('touchend', touchEndHandler);
      canvas.removeEventListener('touchcancel', touchEndHandler);
    };
  }, [currentTool, isDragging, isRotating, selectedPoint, lineStartPoint, brushSize]);


  const handleAddRobotPosition = () => {
    const tf2js = TF2JS.getInstance();
    const mapFrame = 'map';
    const baseFrame = 'base_link';
    const transform = tf2js.findTransform(mapFrame, baseFrame);

    if (!transform) {
      toast.error('无法获取机器人位置，请检查TF连接');
      return;
    }

    const robotEuler = new THREE.Euler();
    robotEuler.setFromQuaternion(transform.rotation, 'XYZ');
    const robotTheta = robotEuler.z;

    const mapManager = mapManagerRef.current;
    const existingPoints = mapManager.getTopologyPoints();
    let pointIndex = existingPoints.length;
    let pointName = `NAV_POINT_${pointIndex}`;
    while (existingPoints.some(p => p.name === pointName)) {
      pointIndex++;
      pointName = `NAV_POINT_${pointIndex}`;
    }

    const newPoint: TopoPoint = {
      name: pointName,
      x: transform.translation.x,
      y: transform.translation.y,
      theta: robotTheta,
      type: 0,
    };

    const command = new AddPointCommand(mapManager, newPoint, updateTopoMap);
    commandManagerRef.current.executeCommand(command);
    setSelectedPoint(newPoint);
    const topoLayer = layerManagerRef.current?.getLayer('topology');
    if (topoLayer instanceof TopoLayer) {
      topoLayer.setSelectedPoint(newPoint);
    }
    toast.success(`已添加机器人当前位置为导航点: ${pointName}`);
  };


  const handlePublishGridToMap = async () => {
    const grid = occupancyGridLayerRef.current?.getMapMessage();
    if (!grid || !grid.info || !grid.data) {
      toast.error("没有可保存的栅格地图数据");
      return;
    }
    try {
      // 优先走网关 API（会写入 PGM+PNG 并更新 telemetry），rosbridge 直连作为备用
      if (gatewayToken) {
        const { apiBase } = await import('../api/gatewayApi');
        const res = await fetch(`${apiBase}/api/map/publish`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            Authorization: `Bearer ${gatewayToken}`,
          },
          body: JSON.stringify({
            width: grid.info.width,
            height: grid.info.height,
            resolution: grid.info.resolution,
            origin: { x: grid.info.origin.position.x, y: grid.info.origin.position.y },
            data: Array.isArray(grid.data) ? grid.data : Array.from(grid.data),
          }),
        });
        if (!res.ok) throw new Error(await res.text());
      } else if (connection?.isConnected()) {
        publishGridToMap(connection, grid);
      } else {
        toast.error("未连接到 rosbridge，请先登录网关或连接 ROS");
        return;
      }
      toast.success("已发布到 /map 话题");
    } catch (error) {
      console.error("发布栅格地图失败:", error);
      toast.error("发布失败: " + (error instanceof Error ? error.message : "未知错误"));
    }
  };


  const handleResetView = () => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!camera || !controls) return;
    yawAngleRef.current = 0;
    camera.up.set(0, 0, 1);
    controls.update();
    camera.position.set(controls.target.x, controls.target.y, camera.position.z);
    // yawAngleRef 已在前面置零，controls.update() 恢复俯视姿态
  };

  const handleEditingPointChange = (field: keyof TopoPoint, value: string | number) => {
    if (!editingPoint) return;
    setEditingPoint({
      ...editingPoint,
      [field]: value,
    });
  };

  const handlePointConfirm = () => {
    if (!editingPoint || !selectedPoint) return;

    const oldPoint = { ...selectedPoint };
    const command = new ModifyPointCommand(mapManagerRef.current, oldPoint, editingPoint, updateTopoMap);
    commandManagerRef.current.executeCommand(command);
    setSelectedPoint(null);
    setEditingPoint(null);
    const topoLayer = layerManagerRef.current?.getLayer('topology');
    if (topoLayer instanceof TopoLayer) {
      topoLayer.setSelectedPoint(null);
    }
  };

  const handlePointCancel = () => {
    setSelectedPoint(null);
    setEditingPoint(null);
    const topoLayer = layerManagerRef.current?.getLayer('topology');
    if (topoLayer instanceof TopoLayer) {
      topoLayer.setSelectedPoint(null);
    }
  };

  const handleFillRobotPosition = () => {
    if (!editingPoint || !robotPos) {
      toast.error('无法获取机器人位置，请检查TF连接');
      return;
    }

    setEditingPoint({
      ...editingPoint,
      x: robotPos.x,
      y: robotPos.y,
      theta: robotPos.theta,
    });
  };

  return (
    <div className="MapEditor">
      <div className="EditorHeader">
        <h2>地图编辑</h2>
        <div style={{ display: 'flex', gap: '10px', alignItems: 'center' }}>
          <button
            className="SaveButton"
            onClick={async () => {
              const points = mapManagerRef.current.getTopologyPoints()
              if (points.length === 0) {
                toast.warning("请先添加点位")
                return
              }
              try {
                // 1) 通过网关直接保存点位到 data/ 目录
                if (gatewayToken) {
                  await fetch(`${import.meta.env.VITE_GATEWAY_URL ?? 'http://localhost:8080'}/api/topology/save`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json', Authorization: `Bearer ${gatewayToken}` },
                    body: JSON.stringify({ points }),
                  })
                }
                // 2) 通过 rosbridge 发布（如果已连接）
                if (connection?.isConnected()) {
                  mapManagerRef.current.publishTopology(connection)
                }
                toast.success(`已保存 ${points.length} 个点位`)
              } catch (error) {
                console.error("发布失败:", error)
                toast.error("发布失败")
              }
            }}
            type="button"
            style={{
              padding: '8px 16px',
              backgroundColor: '#e94560',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '14px',
            }}
            title="发布拓扑任务点到 /map/topology/update"
          >
            📡 发布点位 (/map/topology/update)
          </button>
          <button
            className="SaveButton"
            onClick={handlePublishGridToMap}
            type="button"
            style={{
              padding: "8px 16px",
              backgroundColor: "#e94560",
              color: "white",
              border: "none",
              borderRadius: "4px",
              cursor: "pointer",
              fontSize: "14px",
            }}
            title="发布编辑后的栅格地图到 /map 话题"
          >
            📡 发布到 /map
          </button>
          <button className="CloseButton" onClick={onClose} type="button">
            ×
          </button>
        </div>
      </div>
      <div className="EditorContent">
        <div className="Toolbar toolbar-open">
          <button
              className={`ToolButton ${currentTool === 'move' ? 'active' : ''}`}
              onClick={() => {
                setCurrentTool('move');
                setLineStartPoint(null);
                clearPreviewLine();
                setMousePosition(null);
              }}
              type="button"
              title="移动工具"
            >
              🖱️ 移动
            </button>
            <button
              className={`ToolButton ${currentTool === 'addPoint' ? 'active' : ''}`}
              onClick={() => {
                setCurrentTool('addPoint');
                setLineStartPoint(null);
                clearPreviewLine();
                setMousePosition(null);
              }}
              type="button"
              title="添加拓扑点位"
            >
              ➕ 添加点位
            </button>
            <button
              className={`ToolButton ${currentTool === 'brush' ? 'active' : ''}`}
              onClick={() => {
                setCurrentTool('brush');
                setLineStartPoint(null);
                clearPreviewLine();
                setMousePosition(null);
              }}
              type="button"
              title="绘制障碍物"
            >
              🖌️ 障碍物绘制
            </button>
            <button
              className={`ToolButton ${currentTool === 'eraser' ? 'active' : ''}`}
              onClick={() => {
                setCurrentTool('eraser');
                setLineStartPoint(null);
                clearPreviewLine();
                setMousePosition(null);
              }}
              type="button"
              title="擦除障碍物"
            >
              🧹 橡皮擦
            </button>
            <button
              className={`ToolButton ${currentTool === 'drawLine' ? 'active' : ''}`}
              onClick={() => {
                setCurrentTool('drawLine');
                setLineStartPoint(null);
                clearPreviewLine();
                setMousePosition(null);
              }}
              type="button"
              title="直线绘制"
            >
              📏 直线绘制
            </button>
            <button
              className={`ToolButton ${currentTool === 'rect' ? 'active' : ''}`}
              onClick={() => {
                setCurrentTool('rect');
                setLineStartPoint(null);
                clearPreviewLine();
                clearShapePreview();
                setMousePosition(null);
              }}
              type="button"
              title="矩形障碍物"
            >
              ⬛ 矩形绘制
            </button>
            <button
              className={`ToolButton ${currentTool === 'circle' ? 'active' : ''}`}
              onClick={() => {
                setCurrentTool('circle');
                setLineStartPoint(null);
                clearPreviewLine();
                clearShapePreview();
                setMousePosition(null);
              }}
              type="button"
              title="圆形障碍物"
            >
              ⭕ 圆形绘制
            </button>
            <button
              className={`ToolButton ${currentTool === 'rotateView' ? 'active' : ''}`}
              onClick={() => {
                setCurrentTool('rotateView');
                setLineStartPoint(null);
                clearPreviewLine();
                clearShapePreview();
                setMousePosition(null);
              }}
              type="button"
              title="旋转视角：左键拖拽旋转地图"
            >
              🔄 旋转视角
            </button>
            <button
              className="ToolButton"
              onClick={handleResetView}
              type="button"
              title="重置视角为正北方向"
            >
              🧭 重置视角
            </button>
            <button
              className="ToolButton"
              onClick={() => {
                if (commandManagerRef.current.undo()) {
                  toast.success('撤销');
                }
              }}
              type="button"
              title="撤销上一步 (Ctrl+Z)"
              style={{ marginLeft: 'auto' }}
            >
              ↩ 撤销
            </button>
        </div>
        {(currentTool === 'brush' || currentTool === 'eraser' || currentTool === 'drawLine') && (
          <div className="ToolbarOptions" style={{ padding: '10px', borderTop: '1px solid rgba(255,255,255,0.1)' }}>
            <label style={{ display: 'flex', alignItems: 'center', gap: '10px', fontSize: '14px', color: 'white' }}>
              <span style={{ color: 'white' }}>画笔大小:</span>
              <input
                type="range"
                min="0.05"
                max="1"
                step="0.05"
                value={brushSize}
                onChange={(e) => setBrushSize(parseFloat(e.target.value))}
                style={{ flex: 1 }}
              />
              <span style={{ minWidth: '40px', textAlign: 'right', color: 'white' }}>{brushSize.toFixed(2)}m</span>
            </label>
          </div>
        )}
        {currentTool === 'rotateView' && (
          <div className="ToolbarOptions">
            <p style={{ color: '#94a3b8', fontSize: '13px', margin: 0, padding: '4px 0' }}>
              💡 左键拖拽旋转地图视角
            </p>
          </div>
        )}
        {currentTool === 'addPoint' && (
          <div className="ToolbarOptions">
            <button
              onClick={handleAddRobotPosition}
              type="button"
              style={{
                width: '100%',
                padding: '8px 16px',
                backgroundColor: '#2196F3',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '14px',
                fontWeight: 'bold',
              }}
              title="将机器人当前位置添加为导航点位"
            >
              🤖 添加机器人当前位置
            </button>
          </div>
        )}
        <div className="EditorCanvas">
          <canvas
            ref={canvasRef}
            className={`EditorMapCanvas ${currentTool === 'brush' ? 'cursor-brush' :
                currentTool === 'eraser' ? 'cursor-eraser' :
                currentTool === 'rect' || currentTool === 'circle' || currentTool === 'rotateView' ? 'cursor-crosshair' :
                  ''
              }`}
            onMouseMove={updateBrushIndicator}
            onMouseLeave={() => setMousePosition(null)}
            onContextMenu={(e) => {
              if (currentTool === 'move') {
                e.preventDefault();
              }
            }}
          />
          {(currentTool === 'eraser' || currentTool === 'brush') && mousePosition && (
            <div
              ref={brushIndicatorRef}
              className={`BrushIndicator ${currentTool === 'brush' ? 'BrushIndicator-brush' : 'BrushIndicator-eraser'}`}
              style={{
                left: `${mousePosition.x}px`,
                top: `${mousePosition.y}px`,
                width: `${getBrushIndicatorSize()}px`,
                height: `${getBrushIndicatorSize()}px`,
                marginLeft: `-${getBrushIndicatorSize() / 2}px`,
                marginTop: `-${getBrushIndicatorSize() / 2}px`,
              }}
            />
          )}
        </div>
        {(selectedPoint && editingPoint) && (
          <div className="PropertyPanel">
            {selectedPoint && editingPoint && (
              <div className="PropertySection">
                <h3>点位属性</h3>
                <div className="PropertyRow">
                  <label>名称:</label>
                  <input
                    type="text"
                    value={editingPoint.name}
                    onChange={(e) => handleEditingPointChange('name', e.target.value)}
                  />
                </div>
                <div className="PropertyRow">
                  <label>X:</label>
                  <input
                    type="number"
                    step="0.01"
                    value={editingPoint.x}
                    onChange={(e) => handleEditingPointChange('x', parseFloat(e.target.value) || 0)}
                  />
                </div>
                <div className="PropertyRow">
                  <label>Y:</label>
                  <input
                    type="number"
                    step="0.01"
                    value={editingPoint.y}
                    onChange={(e) => handleEditingPointChange('y', parseFloat(e.target.value) || 0)}
                  />
                </div>
                <div className="PropertyRow">
                  <label>Theta:</label>
                  <input
                    type="number"
                    step="0.01"
                    value={editingPoint.theta}
                    onChange={(e) => handleEditingPointChange('theta', parseFloat(e.target.value) || 0)}
                  />
                </div>
                <button
                  onClick={handleFillRobotPosition}
                  type="button"
                  style={{
                    width: '100%',
                    padding: '8px 16px',
                    marginBottom: '10px',
                    backgroundColor: '#2196F3',
                    color: 'white',
                    border: 'none',
                    borderRadius: '4px',
                    cursor: 'pointer',
                    fontSize: '14px',
                  }}
                  title="将机器人当前位置填入坐标"
                >
                  🤖 填入机器人位置
                </button>
                <div style={{ display: 'flex', gap: '10px', marginBottom: '10px' }}>
                  <button
                    onClick={handlePointConfirm}
                    type="button"
                    style={{
                      flex: 1,
                      padding: '8px 16px',
                      backgroundColor: '#4CAF50',
                      color: 'white',
                      border: 'none',
                      borderRadius: '4px',
                      cursor: 'pointer',
                      fontSize: '14px',
                    }}
                  >
                    确定
                  </button>
                  <button
                    onClick={handlePointCancel}
                    type="button"
                    style={{
                      flex: 1,
                      padding: '8px 16px',
                      backgroundColor: '#555',
                      color: 'white',
                      border: 'none',
                      borderRadius: '4px',
                      cursor: 'pointer',
                      fontSize: '14px',
                    }}
                  >
                    取消
                  </button>
                </div>
                <button
                  className="DeleteButton"
                  onClick={() => {
                    const command = new DeletePointCommand(mapManagerRef.current, selectedPoint, updateTopoMap);
                    commandManagerRef.current.executeCommand(command);
                    setSelectedPoint(null);
                    setEditingPoint(null);
                    const topoLayer = layerManagerRef.current?.getLayer('topology');
                    if (topoLayer instanceof TopoLayer) {
                      topoLayer.setSelectedPoint(null);
                    }
                    toast.success(`已删除点位: ${selectedPoint.name}`);
                  }}
                  type="button"
                >
                  删除点位
                </button>
              </div>
            )}
          </div>
        )}
      </div>
      <div className="CoordinateDisplay">
        <div className="CoordinateRow">
          <span className="CoordinateLabel">鼠标:</span>
          <span className="CoordinateValue">
            {mouseWorldPos
              ? `X: ${mouseWorldPos.x.toFixed(3)}, Y: ${mouseWorldPos.y.toFixed(3)}`
              : '-'}
          </span>
        </div>
        <div className="CoordinateRow">
          <span className="CoordinateLabel">机器人:</span>
          <span className="CoordinateValue">
            {robotPos
              ? `X: ${robotPos.x.toFixed(3)}, Y: ${robotPos.y.toFixed(3)}, θ: ${robotPos.theta.toFixed(3)}`
              : '-'}
          </span>
        </div>
      </div>
    </div>
  );
}
