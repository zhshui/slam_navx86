import { useEffect, useRef, useState } from 'react'
import { useGatewayContext } from '../GatewayProvider'
import { apiBase } from '../../../api/gatewayApi'
import type { NavPoint } from '../../../types/GatewayTypes'

const btnStyle: React.CSSProperties = {
  padding: '6px 12px',
  borderRadius: '4px',
  border: 'none',
  fontSize: '13px',
  cursor: 'pointer',
  color: 'white',
}

export function NavPointPanel() {
  const { snapshot, publishNavGoal, deleteItem, token } = useGatewayContext()
  const [statusText, setStatusText] = useState('')
  const [navOrder, setNavOrder] = useState<string[]>([])
  const [editingNameId, setEditingNameId] = useState<string | null>(null)
  const [editNameValue, setEditNameValue] = useState('')
  const navPoints: NavPoint[] = (snapshot?.navPoints ?? []) as NavPoint[]
  const lastPointNames = useRef('')

  // 导航点变化时同步顺序
  const pointNames = navPoints.map(p => p.name).join(',')
  useEffect(() => {
    if (pointNames && pointNames !== lastPointNames.current) {
      lastPointNames.current = pointNames
      const names = navPoints.map(p => p.name)
      setNavOrder(prev => {
        // 保留已有顺序，新点追加到末尾
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
      setStatusText('顺序已保存' + (snapshot?.runtime?.navStatus === 'running' ? '，导航将按新顺序执行' : ''))
    } catch (e) {
      setStatusText(String(e))
    }
  }

  // 按 navOrder 排序显示（order 为空时按原始顺序）
  const sortedPoints = navOrder.length > 0
    ? (navOrder.map(name => navPoints.find(p => p.name === name)).filter(Boolean) as NavPoint[])
    : navPoints

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

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>导航点管理</h2>

      {statusText && (
        <p style={{ color: statusText.includes('成功') ? '#4CAF50' : '#e94560', marginBottom: '12px' }}>
          {statusText}
        </p>
      )}

      {snapshot?.runtime?.navStatus === 'paused' && (
        <p style={{ color: '#FF9800', marginBottom: '12px', fontSize: '13px' }}>
          ⏸ 导航已暂停 — 用 ▲▼ 调整顺序，点「💾 保存导航顺序」，再「▶ 继续」
        </p>
      )}
      {navPoints.length > 1 && (
        <p style={{ color: '#94a3b8', marginBottom: '8px', fontSize: '12px' }}>
          💡 用每行左侧的 ▲▼ 按钮调整导航顺序，然后点下方「💾 保存导航顺序」
        </p>
      )}
      {navOrder.length > 1 && (
        <div style={{ marginBottom: '12px' }}>
          <button style={{ ...btnStyle, backgroundColor: '#FF9800' }} onClick={saveOrder}>
            💾 保存导航顺序
          </button>
        </div>
      )}
      <table style={{ width: '100%', borderCollapse: 'collapse', color: '#eee' }}>
        <thead>
          <tr style={{ backgroundColor: '#16213e' }}>
            <th style={{ padding: '8px', textAlign: 'left' }}>#</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>名称</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>X</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>Y</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>Yaw</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>操作</th>
          </tr>
        </thead>
        <tbody>
          {sortedPoints.map((p, i) => (
            <tr key={p.id} style={{ borderBottom: '1px solid #333' }}>
              <td style={{ padding: '8px', color: '#94a3b8' }}>{i + 1}</td>
              <td style={{ padding: '8px' }}>
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
                      width: '120px',
                      padding: '4px 6px',
                      borderRadius: '3px',
                      border: '1px solid #e94560',
                      backgroundColor: '#0f3460',
                      color: '#ff3333',
                      fontSize: '13px',
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
              <td style={{ padding: '8px' }}>{p.x.toFixed(2)}</td>
              <td style={{ padding: '8px' }}>{p.y.toFixed(2)}</td>
              <td style={{ padding: '8px' }}>{p.yaw.toFixed(2)}</td>
              <td style={{ padding: '8px', display: 'flex', gap: '4px', flexWrap: 'wrap', alignItems: 'center' }}>
                <button style={{ ...btnStyle, backgroundColor: '#9C27B0', fontSize: '12px', padding: '4px 8px' }}
                  disabled={i === 0} onClick={() => moveUp(i)} title="上移">▲</button>
                <button style={{ ...btnStyle, backgroundColor: '#9C27B0', fontSize: '12px', padding: '4px 8px' }}
                  disabled={i === sortedPoints.length - 1} onClick={() => moveDown(i)} title="下移">▼</button>
                <button
                  style={{ ...btnStyle, backgroundColor: '#4CAF50' }}
                  onClick={() => handlePublish(p)}
                >
                  发布目标
                </button>
                <button
                  style={{ ...btnStyle, backgroundColor: '#f44336' }}
                  onClick={() => deleteItem(`/api/nav-points/${p.id}`)}
                >
                  删除
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {navPoints.length === 0 && (
        <p style={{ color: '#aaa', textAlign: 'center', marginTop: '20px' }}>暂无导航点</p>
      )}
    </div>
  )
}
