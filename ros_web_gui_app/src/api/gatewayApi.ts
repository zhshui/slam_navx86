// 默认相对路径，走 Vite proxy → gateway。远程访问时设 VITE_GATEWAY_URL=http://<本机IP>:8080 可直连
const API_BASE = import.meta.env.VITE_GATEWAY_URL || ''

export const apiBase = API_BASE

/** 当 token 无效或过期时抛出的错误 */
export class AuthError extends Error {
  constructor(message: string) {
    super(message)
    this.name = 'AuthError'
  }
}

export async function apiRequest<T>(path: string, token: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`, {
    ...init,
    headers: {
      'Content-Type': 'application/json',
      Authorization: `Bearer ${token}`,
      ...(init?.headers ?? {}),
    },
  })
  if (!res.ok) {
    const body = await res.text()
    if (res.status === 401) {
      throw new AuthError(body || 'Invalid token')
    }
    throw new Error(body || `HTTP ${res.status}`)
  }
  return (await res.json()) as T
}
