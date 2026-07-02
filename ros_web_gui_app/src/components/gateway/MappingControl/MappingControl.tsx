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

export function MappingControl() {
  const { snapshot, runAction, token, refreshSnapshot } = useGatewayContext()
  const [errorText, setErrorText] = useState('')
  const runtime = snapshot?.runtime

  const execute = async (path: string, body?: object) => {
    try {
      setErrorText('')
      if (body && token) {
        await apiRequest(path, token, { method: 'POST', body: JSON.stringify(body) })
      } else {
        await runAction(path)
      }
      await refreshSnapshot()
    } catch (e) {
      setErrorText(String(e))
    }
  }

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>建图控制</h2>
      <div style={{ display: 'flex', gap: '12px', flexWrap: 'wrap', marginBottom: '16px' }}>
        <button
          style={{ ...btnStyle, backgroundColor: '#4CAF50' }}
          onClick={() => execute('/api/mapping/start')}
        >
          启动建图
        </button>
        <button
          style={{ ...btnStyle, backgroundColor: '#FF9800' }}
          onClick={() => execute('/api/mapping/pause')}
        >
          暂停建图
        </button>
        <button
          style={{ ...btnStyle, backgroundColor: '#f44336' }}
          onClick={() => {
            const save = window.confirm('确认停止建图？\n「确定」保存PCD并生成二维栅格地图\n「取消」仅停止不保存')
            execute('/api/mapping/stop', { save })
          }}
        >
          停止建图
        </button>
      </div>
      <p>当前状态: <strong>{runtime?.mappingStatus ?? '-'}</strong></p>
      {!runtime?.mappingPauseSupported && (
        <p style={{ color: '#aaa', fontSize: '13px' }}>
          当前建图程序不支持真实 pause，网关已预留后续接入接口。
        </p>
      )}
      {errorText && <p style={{ color: '#e94560' }}>{errorText}</p>}
    </div>
  )
}
