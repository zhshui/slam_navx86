import { useGatewayContext } from '../GatewayProvider'

function Card({ title, value }: { title: string; value: string }) {
  return (
    <div style={{
      backgroundColor: '#16213e',
      borderRadius: '8px',
      padding: '16px',
      minWidth: '200px',
    }}>
      <h4 style={{ margin: '0 0 8px 0', color: '#aaa', fontSize: '14px' }}>{title}</h4>
      <div style={{ fontSize: '18px', fontWeight: 'bold' }}>{value}</div>
    </div>
  )
}

export function Dashboard() {
  const { snapshot, wsState } = useGatewayContext()
  const runtime = snapshot?.runtime

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>系统仪表盘</h2>
      <div style={{ display: 'flex', gap: '16px', flexWrap: 'wrap' }}>
        <Card
          title="ROS 网关状态"
          value={runtime?.rosConnected ? 'Connected' : 'Disconnected'}
        />
        <Card
          title="WebSocket 状态"
          value={wsState === 'connected' ? '已连接' : wsState === 'connecting' ? '连接中...' : '未连接'}
        />
        <Card
          title="TF 位姿"
          value={runtime ? `${runtime.tfPose.x.toFixed(2)}, ${runtime.tfPose.y.toFixed(2)}` : '-'}
        />
        <Card
          title="当前地图"
          value={snapshot?.maps.find((m) => m.active === 1)?.name ?? '未切换'}
        />
        <Card
          title="建图状态"
          value={runtime?.mappingStatus ?? '-'}
        />
        <Card
          title="断线重连"
          value="启用（指数退避 + snapshot）"
        />
      </div>
    </div>
  )
}
