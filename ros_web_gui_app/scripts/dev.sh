#!/bin/bash
# One-click dev startup: gateway + frontend
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Starting Gateway ==="
cd "$ROOT_DIR/gateway"
if [ ! -f .env ]; then
  cp .env.example .env
  echo "Created gateway/.env from .env.example"
fi
npm run dev &
GATEWAY_PID=$!

echo "=== Starting Frontend ==="
cd "$ROOT_DIR"
npm run dev &
FRONTEND_PID=$!

echo ""
echo "Gateway: http://localhost:8080"
echo "Frontend: http://localhost:5173"
echo "Press Ctrl+C to stop both"

trap "kill $GATEWAY_PID $FRONTEND_PID 2>/dev/null; exit 0" INT TERM
wait
