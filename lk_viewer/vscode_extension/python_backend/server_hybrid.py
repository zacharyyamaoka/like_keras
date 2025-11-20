#!/usr/bin/env python3
"""
    LK Viewer Backend - Hybrid Mode
    
    Can run in two modes:
    1. Integrated: Spawned by VS Code, reads filesystem directly
    2. Standalone: Run independently, receives code via API
"""

# PYTHON
import sys
import os
from pathlib import Path
from typing import Any, Optional
from fastapi import FastAPI, Body
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configuration
WORKSPACE_ROOT = None
MODE = "integrated"  # or "standalone"


class CodeSubmission(BaseModel):
    """For standalone mode: receive code via API"""
    files: list[dict[str, str]]  # [{"path": "...", "content": "..."}]


def find_python_files(root_path: str, max_files: int = 20) -> list[dict[str, Any]]:
    """INTEGRATED MODE: Read files from filesystem"""
    root = Path(root_path)
    files = []
    
    lk_dir = root / "lk"
    if lk_dir.exists():
        for py_file in lk_dir.rglob("*.py"):
            if '__pycache__' in str(py_file) or 'test_' in py_file.name:
                continue
            
            try:
                if py_file.stat().st_size < 50:
                    continue
                    
                files.append({
                    'path': str(py_file.absolute()),
                    'name': py_file.stem,
                    'relative_path': str(py_file.relative_to(root))
                })
                
                if len(files) >= max_files:
                    break
            except Exception as e:
                print(f"Error reading {py_file}: {e}", file=sys.stderr)
                continue
    
    return files


def build_graph_from_files(files: list[dict[str, Any]]) -> dict[str, Any]:
    """Build React Flow graph"""
    nodes = []
    edges = []
    
    for idx, file_info in enumerate(files):
        node_type = 'default'
        emoji = '📄'
        
        # Classify files
        name_lower = file_info['name'].lower()
        if 'system' in name_lower:
            node_type = 'input'
            emoji = '🤖'
        elif 'agent' in name_lower:
            node_type = 'output'
            emoji = '🎬'
        elif 'sensor' in name_lower:
            emoji = '📡'
        elif 'robot' in name_lower:
            emoji = '🤖'
        elif 'world' in name_lower:
            emoji = '🌍'
        
        label = f"{emoji} {file_info['name']}"
        
        # Grid layout
        col = idx % 5
        row = idx // 5
        
        nodes.append({
            'id': f'file_{idx}',
            'type': node_type,
            'data': {
                'label': label,
                'filePath': file_info.get('path', ''),
                'line': 1,
                'relativePath': file_info.get('relative_path', file_info['name'])
            },
            'position': {
                'x': col * 200 + 50,
                'y': row * 150 + 50
            }
        })
        
        # Connect every 3rd node
        if idx > 0 and idx % 3 == 0:
            edges.append({
                'id': f'e_{idx-1}_{idx}',
                'source': f'file_{idx-1}',
                'target': f'file_{idx}',
                'animated': True
            })
    
    return {'nodes': nodes, 'edges': edges}


@app.get("/")
async def root():
    return {
        "status": "LK Viewer Backend Running ✅",
        "mode": MODE,
        "workspace": WORKSPACE_ROOT if MODE == "integrated" else "N/A (receiving code via API)"
    }


@app.get("/parse")
async def parse_integrated():
    """INTEGRATED MODE: Parse files from filesystem"""
    if MODE != "integrated":
        return {"error": "Not in integrated mode. Use POST /parse with code."}
    
    if not WORKSPACE_ROOT:
        return {"error": "No workspace root set"}
    
    print(f"[Backend] Parsing workspace: {WORKSPACE_ROOT}", file=sys.stderr)
    
    files = find_python_files(WORKSPACE_ROOT, max_files=20)
    
    print(f"[Backend] Found {len(files)} Python files", file=sys.stderr)
    
    if not files:
        return {
            "error": "No Python files found",
            "files_count": 0,
            "graph": {"nodes": [], "edges": []}
        }
    
    graph = build_graph_from_files(files)
    
    print(f"[Backend] Built graph: {len(graph['nodes'])} nodes, {len(graph['edges'])} edges", file=sys.stderr)
    
    return {
        "mode": "integrated",
        "files_count": len(files),
        "graph": graph,
        "workspace": WORKSPACE_ROOT
    }


@app.post("/parse")
async def parse_standalone(submission: CodeSubmission):
    """STANDALONE MODE: Parse code sent via API"""
    print(f"[Backend] Received {len(submission.files)} files via API", file=sys.stderr)
    
    # Convert submitted code to file structure
    files = []
    for idx, file_data in enumerate(submission.files):
        files.append({
            'name': Path(file_data['path']).stem,
            'path': file_data['path'],  # Virtual path
            'relative_path': file_data['path']
        })
    
    graph = build_graph_from_files(files)
    
    return {
        "mode": "standalone",
        "files_count": len(files),
        "graph": graph
    }


@app.get("/mode")
async def get_mode():
    """Check current mode"""
    return {"mode": MODE, "workspace": WORKSPACE_ROOT}


if __name__ == "__main__":
    # Determine mode based on arguments
    if len(sys.argv) > 1:
        WORKSPACE_ROOT = sys.argv[1]
        MODE = "integrated"
        print(f"[Backend] Starting in INTEGRATED mode", file=sys.stderr)
        print(f"[Backend] Workspace: {WORKSPACE_ROOT}", file=sys.stderr)
    else:
        MODE = "standalone"
        print(f"[Backend] Starting in STANDALONE mode", file=sys.stderr)
        print(f"[Backend] Send code via: POST http://localhost:8765/parse", file=sys.stderr)
    
    print(f"[Backend] Server starting on http://localhost:8765", file=sys.stderr)
    print(f"[Backend] Try: curl http://localhost:8765/", file=sys.stderr)
    
    uvicorn.run(app, host="127.0.0.1", port=8765, log_level="info")


