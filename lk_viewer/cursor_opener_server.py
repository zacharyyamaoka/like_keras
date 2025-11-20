#!/usr/bin/env python3
"""
    FastAPI server for opening files in Cursor
    
    Provides a local HTTP endpoint to open files instantly,
    bypassing browser protocol handler delays.
"""

# PYTHON
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import subprocess
import time
import os
from pathlib import Path
import glob

app = FastAPI(title="Cursor File Opener")

# Enable CORS for browser requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

def find_cursor_binary():
    """Find the running Cursor binary (fast!)"""
    mounts = glob.glob("/tmp/.mount_Cursor*/usr/share/cursor/cursor")
    if mounts and os.path.isfile(mounts[0]):
        return mounts[0]
    return "/opt/Cursor.AppImage"

@app.get("/")
async def root():
    return {
        "message": "Cursor File Opener API",
        "endpoints": {
            "/open": "Open a file in Cursor (GET/POST)",
            "/health": "Health check"
        }
    }

@app.get("/health")
async def health():
    return {"status": "ok", "timestamp": time.time()}

@app.get("/open")
@app.post("/open")
async def open_file(path: str, line: int | None = None):
    """
    Open a file in Cursor - FASTEST method!
    
    Args:
        path: Absolute path to the file
        line: Optional line number to jump to
    
    Example:
        GET /open?path=/home/user/file.py&line=42
    """
    start_time = time.perf_counter()
    
    # Validate path exists
    if not os.path.exists(path):
        raise HTTPException(status_code=404, detail=f"File not found: {path}")
    
    try:
        # Find the cursor binary (takes ~1ms via glob)
        binary_start = time.perf_counter()
        cursor_bin = find_cursor_binary()
        binary_find_time = (time.perf_counter() - binary_start) * 1000
        
        # Build the target (VSCode/Cursor format: file:line)
        if line is not None:
            target = f"{path}:{line}"
        else:
            target = path
        
        # Time the cursor call
        cursor_start = time.perf_counter()
        
        # Call Cursor directly - using format from GitHub issue #1858
        # https://github.com/cursor/cursor/issues/1858
        # Format: cursor --reuse-window file:line
        cmd = [cursor_bin, '--no-sandbox', '--reuse-window', target]
        
        # Use Popen to background immediately without waiting
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        cursor_call_time = (time.perf_counter() - cursor_start) * 1000
        total_elapsed = (time.perf_counter() - start_time) * 1000
        
        return {
            "status": "success",
            "path": path,
            "line": line,
            "timings": {
                "find_binary_ms": round(binary_find_time, 3),
                "cursor_call_ms": round(cursor_call_time, 3),
                "total_server_ms": round(total_elapsed, 3),
                "note": "Server returns immediately. Cursor UI may take 200-500ms to switch files."
            },
            "method": "direct_binary_call"
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    print("Starting Cursor File Opener server...")
    print("Open http://localhost:8765 in your browser")
    uvicorn.run(app, host="127.0.0.1", port=8765, log_level="info")

