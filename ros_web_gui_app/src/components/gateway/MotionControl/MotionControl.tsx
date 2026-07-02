import { useGatewayContext } from '../GatewayProvider'
import { useState } from 'react'
import { apiRequest } from '../../../api/gatewayApi'

const btnStyle: React.CSSProperties = {
  padding: '10px 20px',
  borderRadius: '6px',
  border: 'none',
  fontSize: '14px',
  cursor: 'pointer',
  fontWeight: 'bold',
  color: 'white',
}

export function MotionControl() {
  const { snapshot, token, refreshSnapshot } = useGatewayContext()
  const [errorText, setErrorText] = useState('')
  const runtime = snapshot?.runtime

  const motionStatusText: Record<string, string> = {
    running: '运行中',
    stopped: '已停止',
    idle: '空闲',
  }

  const execute = async (command: string) => {
    if (!token) return
    try {
      setErrorText('')
      await apiRequest(`/api/motion/${command}`, token, { method: 'POST' })
      await refreshSnapshot()
    } catch (e) {
      setErrorText(String(e))
    }
  }

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>运动控制</h2>
      <p style={{ fontSize: '13px', color: '#94a3b8', marginBottom: '16px' }}>
        启动/停止 Go2 机器人运动控制节点
      </p>
      <div style={{ display: 'flex', gap: '12px', flexWrap: 'wrap', marginBottom: '16px' }}>
        <button style={{ ...btnStyle, backgroundColor: '#4CAF50' }} onClick={() => execute('start')}>
          ▶ 启动
        </button>
        <button style={{ ...btnStyle, backgroundColor: '#f44336' }} onClick={() => execute('stop')}>
          ⏹ 停止
        </button>
      </div>
      <p>当前状态: <strong style={{
        color: runtime?.motionStatus === 'running' ? '#4CAF50' : runtime?.motionStatus === 'stopped' ? '#FF9800' : '#94a3b8'
      }}>
        {motionStatusText[runtime?.motionStatus ?? 'idle'] ?? runtime?.motionStatus ?? '-'}
      </strong></p>
      {errorText && <p style={{ color: '#e94560' }}>{errorText}</p>}
    </div>
  )
}
