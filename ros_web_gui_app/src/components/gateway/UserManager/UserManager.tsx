import { useEffect, useState } from 'react'
import { useGatewayContext } from '../GatewayProvider'
import { apiRequest } from '../../../api/gatewayApi'
import type { User } from '../../../types/GatewayTypes'

const btnStyle: React.CSSProperties = {
  padding: '6px 14px',
  borderRadius: '4px',
  border: 'none',
  fontSize: '13px',
  cursor: 'pointer',
  color: 'white',
}

export function UserManager() {
  const { token } = useGatewayContext()
  const [users, setUsers] = useState<User[]>([])
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [role, setRole] = useState<'admin' | 'operator'>('operator')
  const [errorText, setErrorText] = useState('')
  const t = token ?? ''

  const refreshUsers = async () => {
    try {
      const res = await apiRequest<{ users: User[] }>('/api/users', t)
      setUsers(res.users)
    } catch (e) {
      setErrorText(String(e))
    }
  }

  useEffect(() => {
    void refreshUsers()
  }, [t])

  return (
    <div>
      <h2 style={{ marginTop: 0 }}>用户管理</h2>

      <div style={{ display: 'flex', gap: '10px', marginBottom: '16px', flexWrap: 'wrap' }}>
        <input
          value={username}
          onChange={(e) => setUsername(e.target.value)}
          placeholder="用户名"
          style={{
            padding: '8px',
            borderRadius: '4px',
            border: '1px solid #333',
            backgroundColor: '#0f3460',
            color: '#eee',
          }}
        />
        <input
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          placeholder="密码"
          type="password"
          style={{
            padding: '8px',
            borderRadius: '4px',
            border: '1px solid #333',
            backgroundColor: '#0f3460',
            color: '#eee',
          }}
        />
        <select
          value={role}
          onChange={(e) => setRole(e.target.value as 'admin' | 'operator')}
          style={{
            padding: '8px',
            borderRadius: '4px',
            border: '1px solid #333',
            backgroundColor: '#0f3460',
            color: '#eee',
          }}
        >
          <option value="operator">operator</option>
          <option value="admin">admin</option>
        </select>
        <button
          style={{ ...btnStyle, backgroundColor: '#4CAF50' }}
          onClick={async () => {
            try {
              setErrorText('')
              await apiRequest('/api/users', t, { method: 'POST', body: JSON.stringify({ username, password, role }) })
              setUsername('')
              setPassword('')
              await refreshUsers()
            } catch (e) { setErrorText(String(e)) }
          }}
        >
          创建用户
        </button>
      </div>

      {errorText && <p style={{ color: '#e94560' }}>{errorText}</p>}

      <table style={{ width: '100%', borderCollapse: 'collapse', color: '#eee' }}>
        <thead>
          <tr style={{ backgroundColor: '#16213e' }}>
            <th style={{ padding: '8px', textAlign: 'left' }}>用户名</th>
            <th style={{ padding: '8px', textAlign: 'left' }}>角色</th>
          </tr>
        </thead>
        <tbody>
          {users.map((u) => (
            <tr key={u.id} style={{ borderBottom: '1px solid #333' }}>
              <td style={{ padding: '8px' }}>{u.username}</td>
              <td style={{ padding: '8px' }}>{u.role}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}
