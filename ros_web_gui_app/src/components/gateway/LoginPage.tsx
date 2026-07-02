import { useState } from 'react'
import { apiBase } from '../../api/gatewayApi'

interface LoginPageProps {
  onLogin: (token: string) => void
}

export function LoginPage({ onLogin }: LoginPageProps) {
  const [username, setUsername] = useState('admin')
  const [password, setPassword] = useState('admin123')
  const [err, setErr] = useState('')

  const submit = async (e: React.FormEvent) => {
    e.preventDefault()
    try {
      const res = await fetch(`${apiBase}/api/auth/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username, password }),
      })
      if (!res.ok) throw new Error('登录失败')
      const body = (await res.json()) as { token: string }
      onLogin(body.token)
    } catch (e) {
      setErr(String(e))
    }
  }

  return (
    <div style={{
      display: 'flex',
      justifyContent: 'center',
      alignItems: 'center',
      height: '100vh',
      backgroundColor: '#1a1a2e',
      color: '#eee',
    }}>
      <form onSubmit={submit} style={{
        backgroundColor: '#16213e',
        padding: '40px',
        borderRadius: '12px',
        width: '360px',
        boxShadow: '0 4px 24px rgba(0,0,0,0.4)',
      }}>
        <h1 style={{ textAlign: 'center', marginBottom: '24px' }}>ROS Web 网关登录</h1>
        <div style={{ marginBottom: '16px' }}>
          <label style={{ display: 'block', marginBottom: '6px', fontSize: '14px' }}>用户名</label>
          <input
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            placeholder="username"
            style={{
              width: '100%',
              padding: '10px',
              borderRadius: '6px',
              border: '1px solid #333',
              backgroundColor: '#0f3460',
              color: '#eee',
              fontSize: '16px',
              boxSizing: 'border-box',
            }}
          />
        </div>
        <div style={{ marginBottom: '20px' }}>
          <label style={{ display: 'block', marginBottom: '6px', fontSize: '14px' }}>密码</label>
          <input
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            type="password"
            placeholder="password"
            style={{
              width: '100%',
              padding: '10px',
              borderRadius: '6px',
              border: '1px solid #333',
              backgroundColor: '#0f3460',
              color: '#eee',
              fontSize: '16px',
              boxSizing: 'border-box',
            }}
          />
        </div>
        <button
          type="submit"
          style={{
            width: '100%',
            padding: '12px',
            borderRadius: '6px',
            border: 'none',
            backgroundColor: '#e94560',
            color: 'white',
            fontSize: '16px',
            cursor: 'pointer',
            fontWeight: 'bold',
          }}
        >
          登录
        </button>
        {err && <p style={{ color: '#e94560', marginTop: '12px', textAlign: 'center' }}>{err}</p>}
      </form>
    </div>
  )
}
