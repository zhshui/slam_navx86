import { useEffect, useRef, useState } from 'react'
import { useGatewayContext } from '../GatewayProvider'
import { apiBase, apiRequest } from '../../../api/gatewayApi'
import type { NavPoint } from '../../../types/GatewayTypes'

const btnStyle: React.CSSProperties = {
  padding: '10px 20px',
  borderRadius: '6px',
  border: 'none',
  fontSize: '14px',
  cursor: 'pointer',
  fontWeight: 'bold',
  color: 'white',
}

const smBtn: React.CSSProperties = {
  ...btnStyle,
  padding: '4px 8px',
  fontSize: '11px',
}

export function NavControl() {
  const { snapshot, token, refreshSnapshot, publishNavGoal, deleteItem } = useGatewayContext()
  const [errorText, setErrorText] = useState('')
  const [statusText, setStatusText] = useState('')
  const [navOrder, setNavOrder] = useState<string[]>([])
  const [editingNameId, setEditingNameId] = useState<string | null>(null)
  const [editNameValue, setEditNameValue] = useState('')
  const navPoints: NavPoint[] = (snapshot?.navPoints ?? []) as NavPoint[]
  const lastPointNames = useRef('')
  const runtime = snapshot?.runtime

  const execute = async (command: string) => {
    if (!token) return
    try {
      setErrorText('')
      await apiRequest('/api/nav/command', token, {
        method: 'POST',
        body: JSON.stringify({ command }),
      })
      await refreshSnapshot()
    } catch (e) {
      setErrorText(String(e))
    }
  }

  async function saveName(p: NavPoint, newName: string) {
    if (!token || !newName.trim() || newName.trim() === p.name) {
      setEditingNameId(null)
      return
    }
    try {
      const res = await fetch(`${apiBase}/api/nav-points/${p.id}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json', Authorization: `Bearer ${token}` },
        body: JSON.stringify({ name: newName.trim() }),
      })
      if (!res.ok) throw new Error(await res.text())
      setStatusText(`已重命名: ${p.name} → ${newName.trim()}`)
      setEditingNameId(null)
      await refreshSnapshot()
    } catch (e) {
      setStatusText('重命名失败: ' + String(e))
    }
  }

  const handlePublish = async (p: { x: number; y: number; yaw: number }) => {
    try {
      setStatusText('')
      await publishNavGoal(p)
      setStatusText('目标点发布成功：/move_base/goal')
    } catch (e) {
      setStatusText(String(e))
    }
  }

  // 同步导航点顺序
  const pointNames = navPoints.map(p => p.name).join(',')
  useEffect(() => {
    if (pointNames && pointNames !== lastPointNames.current) {
      lastPointNames.current = pointNames
      setNavOrder(prev => {
        const names = navPoints.map(p => p.name)
        const existing = prev.filter(n => names.includes(n))
        const added = names.filter(n => !prev.includes(n))
        return [...existing, ...added]
      })
    }
  }, [pointNames])

  function moveUp(index: number) {
    if (index <= 0) return
    const newOrder = [...navOrder]
    ;[newOrder[index - 1], newOrder[index]] = [newOrder[index], newOrder[index - 1]]
    setNavOrder(newOrder)
  }

  function moveDown(index: number) {
    if (index >= navOrder.length - 1) return
    const newOrder = [...navOrder]
    ;[newOrder[index + 1], newOrder[index]] = [newOrder[index], newOrder[index + 1]]
    setNavOrder(newOrder)
  }

  async function saveOrder() {
    if (!token || navOrder.length === 0) return
    try {
      setStatusText('')
      const res = await fetch(`${apiBase}/api/nav/reorder`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', Authorization: `Bearer ${token}` },
        body: JSON.stringify({ order: navOrder }),
      })
      if (!res.ok) throw new Error(await res.text())
      setStatusText('顺序已保存' + (runtime?.navStatus === 'running' ? '，导航将按新顺序执行' : ''))
      await refreshSnapshot()
    } catch (e) {
      setStatusText(String(e))
    }
  }

  const sortedPoints = navOrder.length > 0
    ? (navOrder.map(name => navPoints.find(p => p.name === name)).filter(Boolean) as NavPoint[])
    : navPoints

  const navStatusText: Record<string, string> = {
    running: '导航中',
    paused: '已暂停',
    stopped: '已停止',
    idle: '空闲',
  }

  const isRunning = runtime?.navStatus === 'running'
  const isPaused = runtime?.navStatus === 'paused'
  const isMulti = runtime?.navMode === 'multi'

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>导航控制</h2>

      {/* 控制按钮 */}
      <div style={{ display: 'flex', gap: '12px', flexWrap: 'wrap', marginBottom: '12px' }}>
        {!isRunning && !isPaused && (
          <>
            <button style={{ ...btnStyle, backgroundColor: '#2196F3' }} onClick={() => execute('nav-only')}>🎯 单点导航</button>
            <button style={{ ...btnStyle, backgroundColor: '#4CAF50' }} onClick={() => execute('start')}>📍 多点导航</button>
          </>
        )}
        {isRunning && isMulti && (
          <button style={{ ...btnStyle, backgroundColor: '#FF9800' }} onClick={() => execute('pause')}>⏸ 暂停</button>
        )}
        {isPaused && (
          <button style={{ ...btnStyle, backgroundColor: '#4CAF50' }} onClick={() => execute('resume')}>▶ 继续</button>
        )}
        {(isRunning || isPaused) && (
          <button style={{ ...btnStyle, backgroundColor: '#f44336' }} onClick={() => execute('stop')}>⏹ 停止</button>
        )}
      </div>
      <p style={{ marginBottom: '16px' }}>
        当前状态: <strong style={{ color: isRunning ? '#4CAF50' : isPaused ? '#FF9800' : runtime?.navStatus === 'stopped' ? '#FF9800' : '#94a3b8' }}>
          {navStatusText[runtime?.navStatus ?? 'idle'] ?? runtime?.navStatus ?? '-'}
        </strong>
        {runtime?.navMode ? ` (${runtime.navMode === 'multi' ? '多点' : '单点'}模式)` : ''}
      </p>
      <p style={{ marginBottom: '16px', fontSize: '13px' }}>
        定位状态: {(() => {
          const voxelAge = runtime?.lastVoxelAt ? Date.now() - new Date(runtime.lastVoxelAt).getTime() : 99999;
          if (voxelAge < 3000) return <strong style={{ color: '#4CAF50' }}>🟢 就绪</strong>;
          const tfAge = runtime?.lastTfAt ? Date.now() - new Date(runtime.lastTfAt).getTime() : 99999;
          if (tfAge < 2000) return <strong style={{ color: '#FF9800' }}>🟡 仅TF</strong>;
          return <strong style={{ color: '#e94560' }}>🔴 丢失</strong>;
        })()}
        {runtime?.tfPose && (
          <span style={{ color: '#94a3b8', fontSize: '11px', marginLeft: '8px' }}>
            x: {runtime.tfPose.x.toFixed(2)} y: {runtime.tfPose.y.toFixed(2)} yaw: {(runtime.tfPose.yaw * 180 / Math.PI).toFixed(1)}°
          </span>
        )}
      </p>
      {errorText && <p style={{ color: '#e94560' }}>{errorText}</p>}

      {/* 导航点列表 */}
      <h3 style={{ marginTop: '24px', borderTop: '1px solid #333', paddingTop: '16px' }}>导航点</h3>

      {statusText && (
        <p style={{ color: statusText.includes('成功') ? '#4CAF50' : '#e94560', marginBottom: '8px', fontSize: '13px' }}>
          {statusText}
        </p>
      )}

      {isPaused && (
        <p style={{ color: '#FF9800', marginBottom: '8px', fontSize: '13px' }}>
          ⏸ 导航已暂停 — 用 ▲▼ 调整顺序，点「💾 保存顺序」，再「▶ 继续」
        </p>
      )}

      {navPoints.length > 1 && (
        <div style={{ display: 'flex', gap: '10px', marginBottom: '12px', flexWrap: 'wrap', alignItems: 'center' }}>
          <p style={{ color: '#94a3b8', fontSize: '12px', margin: 0 }}>💡 用 ▲▼ 调整顺序</p>
          <button style={{ ...smBtn, backgroundColor: '#FF9800' }} onClick={saveOrder}>💾 保存顺序</button>
        </div>
      )}

      <div className="NavPointsTable">
      <table style={{ width: '100%', borderCollapse: 'collapse', color: '#eee', fontSize: '13px' }}>
        <thead>
          <tr style={{ backgroundColor: '#16213e' }}>
            <th style={{ padding: '6px', textAlign: 'left', width: '28px' }}>#</th>
            <th style={{ padding: '6px', textAlign: 'left' }}>名称</th>
            <th style={{ padding: '6px', textAlign: 'left' }}>X</th>
            <th style={{ padding: '6px', textAlign: 'left' }}>Y</th>
            <th style={{ padding: '6px', textAlign: 'left' }}>操作</th>
          </tr>
        </thead>
        <tbody>
          {sortedPoints.map((p, i) => (
            <tr key={p.id} style={{ borderBottom: '1px solid #333' }}>
              <td style={{ padding: '6px', color: '#94a3b8' }}>{i + 1}</td>
              <td style={{ padding: '6px' }}>
                {editingNameId === p.id ? (
                  <input
                    autoFocus
                    value={editNameValue}
                    onChange={(e) => setEditNameValue(e.target.value)}
                    onBlur={() => saveName(p, editNameValue)}
                    onKeyDown={(e) => {
                      if (e.key === 'Enter') saveName(p, editNameValue)
                      if (e.key === 'Escape') setEditingNameId(null)
                    }}
                    style={{
                      width: '110px',
                      padding: '3px 5px',
                      borderRadius: '3px',
                      border: '1px solid #e94560',
                      backgroundColor: '#0f3460',
                      color: '#ff3333',
                      fontSize: '12px',
                      fontWeight: 'bold',
                    }}
                  />
                ) : (
                  <span
                    onClick={() => {
                      setEditingNameId(p.id)
                      setEditNameValue(p.name)
                    }}
                    style={{ color: '#ff3333', cursor: 'pointer', fontWeight: 'bold', userSelect: 'none' }}
                    title="点击编辑名称"
                  >
                    {p.name}
                  </span>
                )}
              </td>
              <td style={{ padding: '6px' }}>{p.x.toFixed(2)}</td>
              <td style={{ padding: '6px' }}>{p.y.toFixed(2)}</td>
              <td style={{ padding: '6px', display: 'flex', gap: '4px', flexWrap: 'wrap' }}>
                <button style={{ ...smBtn, backgroundColor: '#9C27B0', padding: '3px 6px', fontSize: '11px' }}
                  disabled={i === 0} onClick={() => moveUp(i)}>▲</button>
                <button style={{ ...smBtn, backgroundColor: '#9C27B0', padding: '3px 6px', fontSize: '11px' }}
                  disabled={i === sortedPoints.length - 1} onClick={() => moveDown(i)}>▼</button>
                <button style={{ ...smBtn, backgroundColor: '#4CAF50' }} onClick={() => handlePublish(p)}>发布</button>
                <button style={{ ...smBtn, backgroundColor: '#f44336' }}
                  onClick={() => deleteItem(`/api/nav-points/${p.id}`)}>删除</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      </div>
      {navPoints.length === 0 && (
        <p style={{ color: '#aaa', textAlign: 'center', marginTop: '16px' }}>暂无导航点 — 在地图编辑中添加并发布</p>
      )}
    </div>
  )
}
