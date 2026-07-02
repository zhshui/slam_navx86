import { useState } from 'react';
import { RosbridgeConnection } from '../utils/RosbridgeConnection';
import { toast } from 'react-toastify';
import * as THREE from 'three';

interface TopoPoint {
  name: string;
  x: number;
  y: number;
  theta: number;
}

interface TopoRoute {
  from_point: string;
  to_point: string;
  route_info: {
    controller: string;
    goal_checker: string;
    speed_limit: number;
  };
}

interface TopoPointInfoPanelProps {
  selectedPoint: TopoPoint | null;
  selectedRoute: TopoRoute | null;
  onClose: () => void;
  connection: RosbridgeConnection;
}

export function TopoPointInfoPanel({
  selectedPoint,
  selectedRoute,
  onClose,
  connection,
}: TopoPointInfoPanelProps) {
  const [gwStatus, setGwStatus] = useState('');

  const gatewayToken = typeof window !== 'undefined' ? localStorage.getItem('gatewayToken') : null;

  const handleNavigateToPoint = () => {
    if (!selectedPoint || !connection.isConnected()) {
      return;
    }

    const quaternion = new THREE.Quaternion();
    quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), selectedPoint.theta);

    const message = {
      header: {
        stamp: {
          sec: Math.floor(Date.now() / 1000),
          nanosec: (Date.now() % 1000) * 1000000,
        },
        frame_id: 'map',
      },
      pose: {
        position: {
          x: selectedPoint.x,
          y: selectedPoint.y,
          z: 0,
        },
        orientation: {
          x: quaternion.x,
          y: quaternion.y,
          z: quaternion.z,
          w: quaternion.w,
        },
      },
    };

    connection.publish('/goal_pose', 'geometry_msgs/msg/PoseStamped', message);
    toast.success(`已发布导航目标: ${selectedPoint.name}`);
  };

  const handleNavigateViaGateway = async () => {
    if (!selectedPoint || !gatewayToken) return;
    try {
      setGwStatus('');
      const gwUrl = import.meta.env.VITE_GATEWAY_URL ?? 'http://localhost:8080';
      const res = await fetch(`${gwUrl}/api/nav/goal`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${gatewayToken}`,
        },
        body: JSON.stringify({
          x: selectedPoint.x,
          y: selectedPoint.y,
          yaw: selectedPoint.theta,
          frame_id: 'map',
        }),
      });
      if (!res.ok) throw new Error(await res.text());
      setGwStatus('✅ 目标已发布到 /move_base/goal');
      toast.success(`导航目标已发布: ${selectedPoint.name}`);
    } catch (e) {
      setGwStatus(`❌ ${String(e)}`);
      toast.error(`发布失败: ${String(e)}`);
    }
  };

  if (selectedPoint) {
    return (
      <div className="TopoPointInfoPanel">
        <div className="TopoPointInfoHeader">
          <h3>导航点信息</h3>
          <button className="CloseButton" onClick={onClose} type="button">
            ×
          </button>
        </div>
        <div className="TopoPointInfoContent">
          <div className="InfoRow">
            <span className="InfoLabel">名称:</span>
            <span className="InfoValue">{selectedPoint.name}</span>
          </div>
          <div className="InfoRow">
            <span className="InfoLabel">X:</span>
            <span className="InfoValue">{selectedPoint.x.toFixed(3)}</span>
          </div>
          <div className="InfoRow">
            <span className="InfoLabel">Y:</span>
            <span className="InfoValue">{selectedPoint.y.toFixed(3)}</span>
          </div>
          <div className="InfoRow">
            <span className="InfoLabel">Theta:</span>
            <span className="InfoValue">{selectedPoint.theta.toFixed(3)}</span>
          </div>
          <button className="NavigateButton" onClick={handleNavigateToPoint} type="button">
            单点导航 (/goal_pose)
          </button>
          {gatewayToken && (
            <>
              <button
                className="NavigateButton"
                onClick={handleNavigateViaGateway}
                type="button"
                style={{ backgroundColor: '#e94560', marginTop: '8px' }}
              >
                网关导航 (/move_base/goal)
              </button>
              {gwStatus && (
                <p style={{
                  fontSize: '12px',
                  marginTop: '6px',
                  color: gwStatus.startsWith('✅') ? '#4CAF50' : '#e94560',
                }}>
                  {gwStatus}
                </p>
              )}
            </>
          )}
        </div>
      </div>
    );
  }

  if (selectedRoute) {
    return (
      <div className="TopoPointInfoPanel">
        <div className="TopoPointInfoHeader">
          <h3>路线信息</h3>
          <button className="CloseButton" onClick={onClose} type="button">
            ×
          </button>
        </div>
        <div className="TopoPointInfoContent">
          <div className="InfoRow">
            <span className="InfoLabel">起点:</span>
            <span className="InfoValue">{selectedRoute.from_point}</span>
          </div>
          <div className="InfoRow">
            <span className="InfoLabel">终点:</span>
            <span className="InfoValue">{selectedRoute.to_point}</span>
          </div>
          <div className="InfoRow">
            <span className="InfoLabel">控制器:</span>
            <span className="InfoValue">{selectedRoute.route_info.controller || '-'}</span>
          </div>
          <div className="InfoRow">
            <span className="InfoLabel">目标检查器:</span>
            <span className="InfoValue">{selectedRoute.route_info.goal_checker || '-'}</span>
          </div>
          <div className="InfoRow">
            <span className="InfoLabel">速度限制:</span>
            <span className="InfoValue">{selectedRoute.route_info.speed_limit.toFixed(2)} m/s</span>
          </div>
        </div>
      </div>
    );
  }

  return null;
}
