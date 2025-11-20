#!/bin/bash

# Launch script for React Flow app with FastAPI backend

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REACT_APP_DIR="${SCRIPT_DIR}/react_flow_app"

echo "🚀 Starting React Flow Application..."
echo ""

# Check if node_modules exists, if not install dependencies
if [ ! -d "${REACT_APP_DIR}/node_modules" ]; then
    echo "📦 Installing React app dependencies..."
    cd "${REACT_APP_DIR}"
    npm install
    cd "${SCRIPT_DIR}"
fi

# Start FastAPI server in background
echo "🔧 Starting FastAPI server on port 8000..."
python3 "${SCRIPT_DIR}/viewer_server.py" &
FASTAPI_PID=$!

# Wait a moment for FastAPI to start
sleep 2

# Start React dev server
echo "⚛️  Starting React dev server on port 3000..."
cd "${REACT_APP_DIR}"
npm run dev &
REACT_PID=$!

echo ""
echo "✅ Both servers are running!"
echo "📍 FastAPI: http://localhost:8000"
echo "📍 React App: http://localhost:3000"
echo ""
echo "Press Ctrl+C to stop both servers..."

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down servers..."
    kill $FASTAPI_PID 2>/dev/null || true
    kill $REACT_PID 2>/dev/null || true
    exit 0
}

# Trap Ctrl+C
trap cleanup INT TERM

# Wait for both processes
wait

