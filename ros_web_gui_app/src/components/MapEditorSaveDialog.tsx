import { useEffect, useRef, useState } from 'react';
import { toast } from 'react-toastify';
import { apiRequest } from '../api/gatewayApi';
import type { OccupancyGrid } from '../utils/MapManager';
import './MapEditorSaveDialog.css';

interface PcdInfo {
  path: string;
  name: string;
  folder: string;
}

export function MapEditorSaveDialog({ token, getGrid, onClose }: {
  token: string | null | undefined;
  getGrid: () => OccupancyGrid | null;
  onClose: () => void;
}) {
  const [name, setName] = useState('');
  const [pcds, setPcds] = useState<PcdInfo[]>([]);
  const [pcdPath, setPcdPath] = useState('');
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState('');
  const savingRef = useRef(false);

  useEffect(() => {
    const controller = new AbortController();
    if (!token) {
      setError('请先登录网关，再保存地图');
      setLoading(false);
      return;
    }
    setLoading(true);
    setError('');
    apiRequest<{ pcds: PcdInfo[] }>('/api/maps/pcds', token, { signal: controller.signal })
      .then(result => {
        setPcds(result.pcds);
        if (!result.pcds.length) setError('没有可选的 PCD，请先完成点云建图');
      })
      .catch(err => {
        if (!controller.signal.aborted) setError(`加载 PCD 失败：${String(err)}`);
      })
      .finally(() => {
        if (!controller.signal.aborted) setLoading(false);
      });
    return () => controller.abort();
  }, [token]);

  const save = async () => {
    if (savingRef.current || !token || !name.trim() || !pcdPath) return;
    const grid = getGrid();
    if (!grid || !grid.data.length) {
      setError('没有可保存的栅格地图数据');
      return;
    }
    savingRef.current = true;
    setSaving(true);
    setError('');
    try {
      const q = grid.info.origin.orientation;
      await apiRequest('/api/maps/save', token, {
        method: 'POST',
        body: JSON.stringify({
          name: name.trim(),
          pcd_path: pcdPath,
          data: {
            width: grid.info.width,
            height: grid.info.height,
            resolution: grid.info.resolution,
            origin: {
              x: grid.info.origin.position.x,
              y: grid.info.origin.position.y,
              yaw: Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
            },
            data: Array.from(grid.data),
          },
        }),
      });
      toast.success(`已保存地图「${name.trim()}」（含所选 PCD），可在地图管理中切换使用`);
      onClose();
    } catch (err) {
      setError(`保存失败：${err instanceof Error ? err.message : String(err)}`);
    } finally {
      savingRef.current = false;
      setSaving(false);
    }
  };

  return (
    <div className="MapSaveBackdrop">
      <form className="MapSaveDialog" role="dialog" aria-modal="true" aria-labelledby="map-save-title"
        onSubmit={event => { event.preventDefault(); void save(); }}>
        <h3 id="map-save-title">选择 PCD 保存地图</h3>
        <p>保存当前编辑的栅格地图，并关联所选 PCD。同名地图将被覆盖。</p>
        <label>地图名称
          <input autoFocus required value={name} disabled={saving} placeholder="输入地图名称"
            onChange={event => setName(event.target.value)} />
        </label>
        <label>配套 PCD
          <select required value={pcdPath} disabled={loading || saving} onChange={event => setPcdPath(event.target.value)}>
            <option value="">{loading ? '正在加载 PCD…' : '请选择 PCD'}</option>
            {pcds.map(pcd => <option key={pcd.path} value={pcd.path}>{pcd.folder}/{pcd.name}.pcd</option>)}
          </select>
        </label>
        {error && <p className="MapSaveError" role="alert">{error}</p>}
        <div className="MapSaveActions">
          <button type="button" disabled={saving} onClick={onClose}>取消</button>
          <button type="submit" disabled={loading || saving || !token || !name.trim() || !pcdPath}>
            {saving ? '保存中…' : '保存地图'}
          </button>
        </div>
      </form>
    </div>
  );
}
