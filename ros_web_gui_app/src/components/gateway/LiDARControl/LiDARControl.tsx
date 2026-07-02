import { useGatewayContext } from '../GatewayProvider'
import { useState } from 'react'

const btnStyle: React.CSSProperties = {
  padding: '10px 20px', borderRadius: '6px', border: 'none',
  fontSize: '14px', cursor: 'pointer', fontWeight: 'bold', color: 'white',
}

export function LiDARControl() {
  const { snapshot, runAction } = useGatewayContext()
  const [errorText, setErrorText] = useState('')
  const runtime = snapshot?.runtime

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>雷达控制</h2>
      <div style={{ display: 'flex', gap: '12px', flexWrap: 'wrap', marginBottom: '16px' }}>
        <button style={{ ...btnStyle, backgroundColor: '#4CAF50' }} onClick={() => runAction('/api/lidar/start')}>
          启动雷达
        </button>
        <button style={{ ...btnStyle, backgroundColor: '#f44336' }} onClick={() => runAction('/api/lidar/stop')}>
          停止雷达
        </button>
      </div>
      <p>当前状态: <strong>{runtime?.lidarStatus ?? '-'}</strong></p>
      {errorText && <p style={{ color: '#e94560' }}>{errorText}</p>}
    </div>
  )
}
