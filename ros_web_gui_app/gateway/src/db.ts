import Database from 'better-sqlite3'
import bcrypt from 'bcryptjs'
import { randomUUID } from 'node:crypto'
import { mkdirSync } from 'node:fs'
import path from 'node:path'

const dbPath = path.resolve(process.cwd(), 'data', 'nav_web.sqlite')
mkdirSync(path.dirname(dbPath), { recursive: true })
const db = new Database(dbPath)

db.pragma('journal_mode = WAL')

// migration: add pcd_path to existing maps table
try {
  db.exec(`ALTER TABLE maps ADD COLUMN pcd_path TEXT NOT NULL DEFAULT ''`)
} catch { /* column already exists */ }

db.exec(`
CREATE TABLE IF NOT EXISTS users (
  id TEXT PRIMARY KEY,
  username TEXT UNIQUE NOT NULL,
  role TEXT NOT NULL,
  password_hash TEXT NOT NULL,
  created_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS maps (
  id TEXT PRIMARY KEY,
  name TEXT UNIQUE NOT NULL,
  yaml_path TEXT NOT NULL,
  pcd_path TEXT NOT NULL DEFAULT '',
  created_at TEXT NOT NULL,
  active INTEGER NOT NULL DEFAULT 0
);
CREATE TABLE IF NOT EXISTS nav_points (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL,
  x REAL NOT NULL,
  y REAL NOT NULL,
  yaw REAL NOT NULL,
  map_id TEXT,
  created_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS virtual_obstacles (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL,
  shape TEXT NOT NULL,
  data TEXT NOT NULL,
  enabled INTEGER NOT NULL DEFAULT 1,
  map_id TEXT,
  created_at TEXT NOT NULL
);
`)

const hasAdmin = db
  .prepare('SELECT COUNT(*) as c FROM users WHERE role = ?')
  .get('admin') as { c: number }

if (hasAdmin.c === 0) {
  db.prepare('INSERT INTO users (id, username, role, password_hash, created_at) VALUES (?, ?, ?, ?, ?)').run(
    randomUUID(),
    'admin',
    'admin',
    bcrypt.hashSync('admin123', 10),
    new Date().toISOString(),
  )
}

const dbExport = db as Database.Database
export default dbExport
