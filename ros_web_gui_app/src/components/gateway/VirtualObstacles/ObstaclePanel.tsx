import { useGatewayContext } from '../GatewayProvider'
import type { VirtualObstacle } from '../../../types/GatewayTypes'

const btnStyle: React.CSSProperties = {
  padding: '6px 12px',
  borderRadius: '4px',
  border: 'none',
  fontSize: '13px',
  cursor: 'pointer',
  color: 'white',
}

export function ObstaclePanel() {
  const { snapshot, toggleObstacle, deleteItem } = useGatewayContext()
  const obstacles: VirtualObstacle[] = (snapshot?.virtualObstacles ?? []) as VirtualObstacle[]

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>虚拟障碍物</h2>

      <div style={{ display: 'flex', gap: '10px', marginBottom: '12px', flexWrap: 'wrap' }}>
        <span style={{ color: '#aaa', fontSize: '13px' }}>
          使用地图编辑器中的工具绘制虚拟障碍物，然后在此管理。
        </span>
      </div>

      <table style={{ width: '100%', borderCollapse: 'collapse', color: '#eee' }}>
        <thead>
          <tr style={{ backgroundColor: '#16213e' }}>
            <th style={{ padding: '8px', textAlign: 'left' }}>名称</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>形状</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>状态</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>操作</th>
          </tr>
        </thead>
        <tbody>
          {obstacles.map((o) => (
            <tr key={o.id} style={{ borderBottom: '1px solid #333' }}>
              <td style={{ padding: '8px' }}>{o.name}</td>
              <td style={{ padding: '8px' }}>{o.shape}</td>
              <td style={{ padding: '8px' }}>{o.enabled ? '✅ 启用' : '❌ 禁用'}</td>
              <td style={{ padding: '8px', display: 'flex', gap: '6px' }}>
                <button
                  style={{ ...btnStyle, backgroundColor: o.enabled ? '#FF9800' : '#4CAF50' }}
                  onClick={() => toggleObstacle(o.id, !o.enabled)}
                >
                  {o.enabled ? '禁用' : '启用'}
                </button>
                <button
                  style={{ ...btnStyle, backgroundColor: '#f44336' }}
                  onClick={() => deleteItem(`/api/virtual-obstacles/${o.id}`)}
                >
                  删除
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {obstacles.length === 0 && (
        <p style={{ color: '#aaa', textAlign: 'center', marginTop: '20px' }}>暂无虚拟障碍物</p>
      )}
    </div>
  )
}
