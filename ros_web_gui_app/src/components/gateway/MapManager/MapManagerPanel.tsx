import { useEffect, useRef, useState } from 'react'
import { useGatewayContext } from '../GatewayProvider'
import { apiBase, apiRequest } from '../../../api/gatewayApi'

interface PcdInfo {
  path: string
  name: string
  folder: string
  mtimeMs: number
  mtime: string
}

const selectStyle: React.CSSProperties = {
  padding: '6px 8px',
  borderRadius: '4px',
  border: '1px solid #333',
  backgroundColor: '#0f3460',
  color: '#eee',
  fontSize: '13px',
  maxWidth: '180px',
}

const btnStyle: React.CSSProperties = {
  padding: '6px 14px',
  borderRadius: '4px',
  border: 'none',
  fontSize: '13px',
  cursor: 'pointer',
  fontWeight: 'bold',
  color: 'white',
}

export function MapManagerPanel() {
  const { token, snapshot, refreshSnapshot } = useGatewayContext()
  const [name, setName] = useState('')
  const [errorText, setErrorText] = useState('')
  const [statusText, setStatusText] = useState('')
  const [switchingId, setSwitchingId] = useState<string | null>(null)
  const [editingName, setEditingName] = useState<{ id: string; name: string } | null>(null)
  const [pcds, setPcds] = useState<PcdInfo[]>([])
  const [savePcd, setSavePcd] = useState('')
  const [importPcd, setImportPcd] = useState('')
  const fileRef = useRef<HTMLInputElement | null>(null)
  const maps = snapshot?.maps ?? []
  const t = token ?? ''

  // 加载可选 PCD 列表
  useEffect(() => {
    fetch(`${apiBase}/api/maps/pcds`, { headers: { Authorization: `Bearer ${t}` } })
      .then(r => r.json())
      .then(d => setPcds(d.pcds || []))
      .catch(() => {})
  }, [t])

  const pcdOptions = (value: string, onChange: (v: string) => void) => (
    <select style={selectStyle} value={value} onChange={e => onChange(e.target.value)}>
      <option value="">自动选择 PCD</option>
      {pcds.map(p => (
        <option key={p.path} value={p.path}>
          {p.folder}/{p.name}.pcd
        </option>
      ))}
    </select>
  )

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>地图管理</h2>

      <div style={{ display: 'flex', gap: '10px', marginBottom: '12px', flexWrap: 'wrap', alignItems: 'center' }}>
        <input
          placeholder="地图名称"
          value={name}
          onChange={(e) => setName(e.target.value)}
          style={{
            padding: '8px',
            borderRadius: '4px',
            border: '1px solid #333',
            backgroundColor: '#0f3460',
            color: '#eee',
          }}
        />
        {pcdOptions(savePcd, setSavePcd)}
        <button style={{ ...btnStyle, backgroundColor: '#4CAF50' }} onClick={async () => {
          try {
            setErrorText('')
            const body: Record<string, string> = { name }
            if (savePcd) body.pcd_path = savePcd
            await apiRequest('/api/maps/save', t, { method: 'POST', body: JSON.stringify(body) })
            setName('')
            setSavePcd('')
            await refreshSnapshot()
          } catch (e) { setErrorText(String(e)) }
        }}>保存当前地图</button>
      </div>

      <div style={{ display: 'flex', gap: '10px', marginBottom: '12px', flexWrap: 'wrap', alignItems: 'center' }}>
        <input type="file" ref={fileRef} accept=".pgm,image/x-portable-graymap" style={{ color: '#eee' }} />
        {pcdOptions(importPcd, setImportPcd)}
        <button style={{ ...btnStyle, backgroundColor: '#FF9800' }} onClick={async () => {
          try {
            const file = fileRef.current?.files?.[0]
            if (!file) return
            const name = file.name.replace(/\.(yaml|yml)$/i, '')
            const form = new FormData()
            form.append('name', name)
            form.append('mapFile', file)
            if (importPcd) form.append('pcd_path', importPcd)
            const res = await fetch(`${apiBase}/api/maps/import`, {
              method: 'POST',
              headers: { Authorization: `Bearer ${t}` },
              body: form,
            })
            if (!res.ok) setErrorText(await res.text())
            setImportPcd('')
            await refreshSnapshot()
          } catch (e) { setErrorText(String(e)) }
        }}>导入</button>
      </div>

      {errorText && <p style={{ color: '#e94560' }}>{errorText}</p>}
      {statusText && <p style={{ color: '#4CAF50' }}>{statusText}</p>}

      <table style={{ width: '100%', borderCollapse: 'collapse', color: '#eee', fontSize: '12px' }}>
        <thead>
          <tr style={{ backgroundColor: '#16213e' }}>
            <th style={{ padding: '5px 6px', textAlign: 'left' }}>名称</th>
            <th style={{ padding: '5px 6px', textAlign: 'left' }}>栅格</th>
            <th style={{ padding: '5px 6px', textAlign: 'left' }}>PCD</th>
            <th style={{ padding: '5px 6px', textAlign: 'left' }}>激活</th>
            <th style={{ padding: '5px 6px', textAlign: 'left', whiteSpace: 'nowrap' }}>操作</th>
          </tr>
        </thead>
        <tbody>
          {maps.map((m) => (
            <tr key={m.id} style={{ borderBottom: '1px solid #333' }}>
              <td style={{ padding: '4px 6px' }}>
                {editingName?.id === m.id ? (
                  <input
                    autoFocus
                    value={editingName.name}
                    onChange={e => setEditingName({ id: m.id, name: e.target.value })}
                    onBlur={async () => {
                      const n = editingName.name.trim()
                      setEditingName(null)
                      if (n && n !== m.name) {
                        try {
                          await apiRequest(`/api/maps/${m.id}/rename`, t, { method: 'PUT', body: JSON.stringify({ name: n }) })
                          await refreshSnapshot()
                          setStatusText(`已重命名为 ${n}`)
                        } catch (e) { setErrorText(String(e)) }
                      }
                    }}
                    onKeyDown={e => { if (e.key === 'Enter') (e.target as HTMLInputElement).blur() }}
                    style={{ width: '80px', padding: '2px', background: '#0f3460', color: '#eee', border: '1px solid #4CAF50', borderRadius: '3px', fontSize: '12px' }}
                  />
                ) : (
                  <span
                    style={{ cursor: 'pointer', borderBottom: '1px dashed #666', fontSize: '12px' }}
                    onClick={() => setEditingName({ id: m.id, name: m.name })}
                    title="点击编辑名称"
                  >{m.name}</span>
                )}
              </td>
              <td style={{ padding: '4px 6px', fontSize: '11px', color: '#aaa' }}>{m.yaml_path.split('/').pop()}</td>
              <td style={{ padding: '4px 6px' }}>
                {m.pcd_path ? (
                  <span style={{ color: '#4CAF50', fontSize: '11px' }}>{m.pcd_path.split('/').pop()}</span>
                ) : (
                  <select
                    style={{ padding: '2px', borderRadius: '3px', backgroundColor: '#0f3460', color: '#eee', border: '1px solid #333', maxWidth: '100px', fontSize: '11px' }}
                    defaultValue=""
                    onChange={async (e) => {
                      const val = e.target.value
                      if (!val) return
                      try {
                        await apiRequest(`/api/maps/${m.id}/pcd`, t, { method: 'PUT', body: JSON.stringify({ pcd_path: val }) })
                        await refreshSnapshot()
                      } catch (e) { setErrorText(String(e)) }
                    }}
                  >
                    <option value="">选择...</option>
                    {maps.filter(x => x.pcd_path).map(x => (
                      <option key={x.id} value={x.pcd_path}>{x.pcd_path.split('/').pop()}</option>
                    ))}
                  </select>
                )}
              </td>
              <td style={{ padding: '4px 6px', fontSize: '12px' }}>{m.active ? '是' : '否'}</td>
              <td style={{ padding: '4px 6px', display: 'flex', gap: '3px' }}>
                <button
                  style={{ ...btnStyle, backgroundColor: switchingId === m.id ? '#666' : '#2196F3', cursor: switchingId ? 'wait' : 'pointer', padding: '3px 6px', fontSize: '11px' }}
                  disabled={!!switchingId}
                  onClick={async () => {
                    setSwitchingId(m.id)
                    setErrorText('')
                    setStatusText('')
                    try {
                      await apiRequest(`/api/maps/${m.id}/switch`, t, { method: 'POST' })
                      await refreshSnapshot()
                      setStatusText(`已切换到「${m.name}」(PCD需重启导航生效)`)
                    } catch (e) {
                      setErrorText(`切换失败: ${e}`)
                    } finally {
                      setSwitchingId(null)
                    }
                  }}
                >{switchingId === m.id ? '切换中...' : '切换'}</button>
                <button style={{ ...btnStyle, backgroundColor: '#9C27B0', padding: '3px 6px', fontSize: '11px' }} onClick={async () => {
                  try {
                    setStatusText('正在打包...')
                    const res = await fetch(`${apiBase}/api/maps/${m.id}/export`, {
                      headers: { Authorization: `Bearer ${t}` },
                    })
                    if (!res.ok) { setErrorText('导出失败'); return }
                    const blob = await res.blob()
                    const url = URL.createObjectURL(blob)
                    const a = document.createElement('a')
                    a.href = url
                    a.download = `${m.name}.zip`
                    a.click()
                    URL.revokeObjectURL(url)
                    setStatusText(`已导出 ${m.name}.zip（含 PCD）`)
                  } catch (e) { setErrorText(`导出: ${e}`) }
                }}>导出</button>
                <button style={{ ...btnStyle, backgroundColor: '#f44336', padding: '3px 6px', fontSize: '11px' }} onClick={() => {
                  apiRequest(`/api/maps/${m.id}`, t, { method: 'DELETE' })
                    .then(() => refreshSnapshot())
                    .catch(e => setErrorText(String(e)))
                }}>删除</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {maps.length === 0 && <p style={{ color: '#aaa', textAlign: 'center', marginTop: '20px' }}>暂无地图</p>}
    </div>
  )
}
