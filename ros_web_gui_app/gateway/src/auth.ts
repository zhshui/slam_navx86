import type { Request, Response, NextFunction } from 'express'
import jwt from 'jsonwebtoken'
import db from './db.js'

const JWT_SECRET = process.env.JWT_SECRET ?? 'nav-web-default-secret'

export interface TokenPayload {
  sub: string
  username: string
  role: 'admin' | 'operator'
}

export interface AuthedRequest extends Request {
  user?: TokenPayload
}

export function signToken(payload: TokenPayload): string {
  return jwt.sign(payload, JWT_SECRET, { expiresIn: '24h' })
}

export function getUserById(sub: string) {
  const user = db.prepare('SELECT id, username, role, created_at FROM users WHERE id = ?').get(sub) as
    | { id: string; username: string; role: 'admin' | 'operator'; created_at: string }
    | undefined
  return user ?? null
}

export function requireAuth(req: Request, res: Response, next: NextFunction) {
  const header = req.headers.authorization
  if (!header || !header.startsWith('Bearer ')) {
    res.status(401).json({ error: 'Unauthorized' })
    return
  }
  try {
    const payload = jwt.verify(header.slice(7), JWT_SECRET) as TokenPayload
    ;(req as AuthedRequest).user = payload
    next()
  } catch {
    res.status(401).json({ error: 'Invalid token' })
  }
}

export function requireRole(roles: string[]) {
  return (req: Request, res: Response, next: NextFunction) => {
    const authed = req as AuthedRequest
    if (!authed.user || !roles.includes(authed.user.role)) {
      res.status(403).json({ error: 'Forbidden' })
      return
    }
    next()
  }
}
