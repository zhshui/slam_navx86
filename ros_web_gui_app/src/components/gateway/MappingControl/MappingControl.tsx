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
  const [stopDialogOpen, setStopDialogOpen] = useState(false)
  const [stopSubmitting, setStopSubmitting] = useState(false)
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
          onClick={() => setStopDialogOpen(true)}
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
      {stopDialogOpen && (
        <div
          style={{
            position: 'fixed',
            inset: 0,
            zIndex: 10000,
            background: 'rgba(0,0,0,0.65)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            padding: '20px',
          }}
        >
          <div
            role="dialog"
            aria-modal="true"
            aria-labelledby="mapping-stop-title"
            style={{
              width: 'min(440px, 100%)',
              borderRadius: '12px',
              background: '#202534',
              color: 'white',
              padding: '24px',
              boxShadow: '0 18px 60px rgba(0,0,0,0.45)',
            }}
          >
            <h3 id="mapping-stop-title" style={{ marginTop: 0 }}>结束建图</h3>
            <p style={{ lineHeight: 1.6 }}>
              是否保存当前点云，并生成二维栅格地图？地图转换可能需要几分钟。
            </p>
            <div style={{ display: 'flex', gap: '10px', justifyContent: 'flex-end', flexWrap: 'wrap', marginTop: '22px' }}>
              <button
                disabled={stopSubmitting}
                style={{ ...btnStyle, backgroundColor: '#606778' }}
                onClick={() => setStopDialogOpen(false)}
              >
                返回
              </button>
              <button
                disabled={stopSubmitting}
                style={{ ...btnStyle, backgroundColor: '#f44336' }}
                onClick={async () => {
                  setStopSubmitting(true)
                  await execute('/api/mapping/stop', { save: false })
                  setStopSubmitting(false)
                  setStopDialogOpen(false)
                }}
              >
                不保存结束
              </button>
              <button
                disabled={stopSubmitting}
                style={{ ...btnStyle, backgroundColor: '#4CAF50' }}
                onClick={async () => {
                  setStopSubmitting(true)
                  await execute('/api/mapping/stop', { save: true })
                  setStopSubmitting(false)
                  setStopDialogOpen(false)
                }}
              >
                {stopSubmitting ? '处理中…' : '保存并结束'}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  )
}
