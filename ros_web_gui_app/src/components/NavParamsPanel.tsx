import { useState, useEffect, useCallback, useRef } from 'react';

// ============================================================
// 导航参数配置面板 — 所有可调参数集中管理
// ============================================================

interface ParamDef {
  key: string;
  label: string;
  min: number;
  max: number;
  step: number;
  default: number;
  unit: string;
  desc: string;
  apply: 'reconfigure' | 'local' | 'rosparam';
}

// 所有参数定义
const PARAM_DEFS: Record<string, ParamDef[]> = {
  '点云显示': [
    { key: 'pc_density',           label: '点云密度',       min: 10000, max: 5000000, step: 10000, default: 5000000, unit: '万', desc: '绿色点云累积总点数上限', apply: 'local' },
    { key: 'scan_distance',        label: '扫描显示距离',   min: 0.1,   max: 5,       step: 0.1,   default: 5,       unit: 'm',  desc: '超过此距离的点云不显示，5=不限', apply: 'local' },
  ],
  '感知过滤': [
    { key: 'min_obstacle_height', label: '高度下限',     min: -2, max: 5, step: 0.1, default: -0.15, unit: 'm',  desc: '低于此高度的点视为地面杂波，滤除', apply: 'rosparam' },
    { key: 'max_obstacle_height', label: '高度上限', min: 0.5, max: 10, step: 0.1, default: 1.5, unit: 'm',  desc: '高于此高度的点视为天花板杂波，滤除', apply: 'rosparam' },
  ],
  '避障策略': [
    { key: 'inflation_radius',     label: '膨胀半径',       min: 0.05, max: 2.0, step: 0.05, default: 0.45, unit: 'm',  desc: '安全膨胀距离（需配合衰减系数）', apply: 'rosparam' },
    { key: 'cost_scaling_factor',  label: '代价衰减系数',   min: 0.5,  max: 20, step: 0.5,  default: 10.0, unit: '',   desc: '代价指数衰减率，越小膨胀越硬、越大越软', apply: 'rosparam' },
  ],
  '到达判断': [
    { key: 'xy_goal_tolerance',    label: '到达精度',       min: 0.10, max: 1.0, step: 0.02, default: 0.10, unit: 'm',  desc: '距目标点此距离内即判定到达', apply: 'reconfigure' },
    { key: 'yaw_goal_tolerance',   label: '朝向精度',       min: 0.05, max: 1.0, step: 0.05, default: 0.15, unit: 'rad', desc: '朝向误差此范围内即判定到达', apply: 'reconfigure' },
  ],
};

const ALL_PARAMS = Object.values(PARAM_DEFS).flat();

const STORAGE_KEY = 'nav_params_values';

function loadSaved(): Record<string, number> {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) return JSON.parse(raw);
  } catch {}
  return {};
}

function saveValues(vals: Record<string, number>) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(vals));
  } catch {}
}

function getDefaults(): Record<string, number> {
  const d: Record<string, number> = {};
  for (const p of ALL_PARAMS) d[p.key] = p.default;
  return d;
}

// ============================================================
// 组件
// ============================================================

interface NavParamsPanelProps {
  gatewayToken: string | null;
  onClose: () => void;
  pcDensity: number;
  scanDistance: number;
  onPcDensityChange: (v: number) => void;
  onScanDistanceChange: (v: number) => void;
}

export function NavParamsPanel({ gatewayToken, onClose, pcDensity, scanDistance, onPcDensityChange, onScanDistanceChange }: NavParamsPanelProps) {
  const panelRef = useRef<HTMLDivElement>(null);

  // 原生触摸滚动支持
  useEffect(() => {
    const panel = panelRef.current;
    if (!panel) return;
    let startY = 0, startScroll = 0;
    const ts = (e: TouchEvent) => { if (e.touches.length === 1) { startY = e.touches[0].clientY; startScroll = panel.scrollTop; } };
    const tm = (e: TouchEvent) => { if (e.touches.length === 1) { panel.scrollTop = startScroll + (startY - e.touches[0].clientY); } };
    panel.addEventListener('touchstart', ts, { passive: true });
    panel.addEventListener('touchmove', tm, { passive: true });
    return () => { panel.removeEventListener('touchstart', ts); panel.removeEventListener('touchmove', tm); };
  }, []);

  const [values, setValues] = useState<Record<string, number>>(() => {
    const saved = loadSaved();
    const defaults = getDefaults();
    // 合并：saved 中有就用，没有用默认
    for (const p of ALL_PARAMS) {
      if (!(p.key in saved)) saved[p.key] = defaults[p.key]!;
    }
    return saved;
  });
  useEffect(() => {
    saveValues(values);
  }, [values]);

  const RESTART_PARAM_KEYS = new Set(['min_obstacle_height', 'max_obstacle_height', 'inflation_radius', 'cost_scaling_factor']);

  const applyParam = useCallback(async (key: string, value: number, apply: ParamDef['apply']) => {
    if (apply === 'local') {
      if (key === 'pc_density') onPcDensityChange(value);
      if (key === 'scan_distance') onScanDistanceChange(value);
      return;
    }
    if (!gatewayToken) return;
    const apiBase = (await import('../api/gatewayApi')).apiBase;
    const headers = { 'Content-Type': 'application/json', Authorization: `Bearer ${gatewayToken}` };
    try {
      if (apply === 'rosparam') {
        await fetch(`${apiBase}/api/nav/param/rosparam`, {
          method: 'POST', headers, body: JSON.stringify({ key, value }),
        });
        if (!RESTART_PARAM_KEYS.has(key)) {
          await fetch(`${apiBase}/api/nav/costmap-clear`, {
            method: 'POST', headers,
          });
        }
      } else {
        // TEB 参数：dynamic_reconfigure 即时生效
        await fetch(`${apiBase}/api/nav/param/reconfigure`, {
          method: 'POST', headers, body: JSON.stringify({ key, value }),
        });
      }
    } catch {}
  }, [gatewayToken, onPcDensityChange, onScanDistanceChange]);

  const handleChange = (key: string, value: number, apply: ParamDef['apply']) => {
    setValues(prev => ({ ...prev, [key]: value }));
    applyParam(key, value, apply);
  };

  const handleResetAll = () => {
    const defaults = getDefaults();
    setValues(defaults);
    for (const p of ALL_PARAMS) {
      applyParam(p.key, p.default, p.apply);
    }
  };

  // ── 导出参数为 JSON 文件 ──
  const handleExport = () => {
    const exportData: Record<string, any> = {
      exportedAt: new Date().toISOString(),
      params: {} as Record<string, number>,
    };
    for (const p of ALL_PARAMS) {
      exportData.params[p.key] = values[p.key] ?? p.default;
    }
    const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `nav_params_${new Date().toISOString().slice(0, 10)}.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  // ── 从 JSON 文件导入 ──
  const handleImport = () => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = async (e) => {
      const file = (e.target as HTMLInputElement).files?.[0];
      if (!file) return;
      try {
        const text = await file.text();
        const data = JSON.parse(text);
        const imported = data.params || data;
        const newValues = { ...values };
        for (const p of ALL_PARAMS) {
          if (typeof imported[p.key] === 'number') {
            newValues[p.key] = imported[p.key];
          }
        }
        setValues(newValues);
        // 逐个写入系统
        for (const p of ALL_PARAMS) {
          const v = imported[p.key];
          if (typeof v === 'number') {
            await applyParam(p.key, v, p.apply);
          }
        }
      } catch (err) {
        console.error('导入失败:', err);
      }
    };
    input.click();
  };

  const param = (key: string) => ALL_PARAMS.find(p => p.key === key)!;

  return (
    <div ref={panelRef} style={{
      position: 'absolute', top: 0, right: 0, bottom: 0, width: '340px',
      backgroundColor: 'rgba(20,20,30,0.95)', color: '#ccc', zIndex: 200,
      overflowY: 'auto', overflowX: 'hidden', padding: '16px', fontSize: '13px',
      paddingBottom: 'calc(16px + env(safe-area-inset-bottom, 0px))',
      borderLeft: '1px solid rgba(255,255,255,0.1)',
      fontFamily: 'system-ui, sans-serif',
      touchAction: 'pan-y',
      overscrollBehavior: 'contain',
      scrollbarWidth: 'thin',
      scrollbarColor: 'rgba(255,255,255,0.25) transparent',
    }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 12 }}>
        <span style={{ fontWeight: 'bold', fontSize: '15px', color: '#fff' }}>⚙️ 导航参数配置</span>
        <button onClick={onClose} style={{ background: 'none', border: 'none', color: '#999', cursor: 'pointer', fontSize: 18 }}>✕</button>
      </div>

      {Object.entries(PARAM_DEFS).map(([group, params]) => (
        <div key={group} style={{ marginBottom: 16 }}>
          <div style={{ color: '#4af', fontSize: 13, fontWeight: 'bold', marginBottom: 8, borderBottom: '1px solid rgba(255,255,255,0.1)', paddingBottom: 4 }}>
            {group}
          </div>
          {params.map(p => {
            const val = values[p.key] ?? p.default;
            const isDefault = Math.abs(val - p.default) < p.step / 2;
            return (
              <div key={p.key} style={{ marginBottom: 10 }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline' }}>
                  <span style={{ color: isDefault ? '#aaa' : '#fff' }}>{p.label}</span>
                  <span style={{
                    fontWeight: 'bold', fontSize: 12,
                    color: isDefault ? '#888' : '#0f0',
                  }}>
                    {p.key === 'pc_density' ? (val / 10000).toFixed(0) + '万'
                      : p.key === 'scan_distance' ? (val >= 5 ? '不限' : val.toFixed(1) + 'm')
                      : p.key === 'cost_scaling_factor' ? val.toFixed(1)
                      : p.step < 1 ? val.toFixed(2) : p.step < 0.1 ? val.toFixed(1) : Math.round(val)}{p.unit}
                  </span>
                </div>
                <input
                  type="range"
                  min={p.min} max={p.max} step={p.step}
                  value={val}
                  onChange={(e) => handleChange(p.key, Number(e.target.value), p.apply)}
                  style={{ width: '100%', accentColor: '#4af', height: 4, cursor: 'pointer', marginTop: 2 }}
                />
                <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 10, color: '#666' }}>
                  <span>{p.min}{p.unit}</span>
                  <span title={p.desc} style={{ cursor: 'help', color: '#555' }}>{p.desc}</span>
                  <span>{p.max}{p.unit}</span>
                </div>
              </div>
            );
          })}
        </div>
      ))}

      <div style={{ display: 'flex', gap: 8, marginTop: 8, flexWrap: 'wrap' }}>
        <button onClick={handleExport} style={{
          flex: 1, minWidth: 70, padding: '8px', borderRadius: 6, border: '1px solid rgba(255,255,255,0.2)',
          backgroundColor: 'rgba(76,175,80,0.15)', color: '#4CAF50', cursor: 'pointer', fontSize: 12,
        }}>
          📤 导出
        </button>
        <button onClick={handleImport} style={{
          flex: 1, minWidth: 70, padding: '8px', borderRadius: 6, border: '1px solid rgba(255,255,255,0.2)',
          backgroundColor: 'rgba(33,150,243,0.15)', color: '#2196F3', cursor: 'pointer', fontSize: 12,
        }}>
          📥 导入
        </button>
        <button onClick={handleResetAll} style={{
          flex: 1, minWidth: 70, padding: '8px', borderRadius: 6, border: '1px solid rgba(255,255,255,0.2)',
          backgroundColor: 'rgba(255,255,255,0.05)', color: '#fff', cursor: 'pointer', fontSize: 12,
        }}>
          🔄 默认值
        </button>
        <button onClick={onClose} style={{
          flex: 1, minWidth: 50, padding: '8px', borderRadius: 6, border: '1px solid rgba(255,255,255,0.2)',
          backgroundColor: 'rgba(255,255,255,0.05)', color: '#fff', cursor: 'pointer', fontSize: 12,
        }}>
          关闭
        </button>
      </div>
    </div>
  );
}
