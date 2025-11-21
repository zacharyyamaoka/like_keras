#!/usr/bin/env python3
"""
LK Viewer Backend Server

Parses Python codebase and provides graph data to VS Code extension.
"""

# PYTHON
import sys
import os
from pathlib import Path
from typing import Any
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

app = FastAPI()

# Allow CORS for VS Code webview
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Store workspace root
WORKSPACE_ROOT = None


def find_python_files(root_path: str, max_files: int = 20) -> list[dict[str, Any]]:
    """Find Python files in the workspace"""
    root = Path(root_path)
    files = []

    # Look for Python files in lk/ directory
    lk_dir = root / "lk"
    if lk_dir.exists():
        for py_file in lk_dir.rglob("*.py"):
            # Skip __pycache__ and test files
            if "__pycache__" in str(py_file) or "test_" in py_file.name:
                continue

            try:
                # Get file size to skip empty files
                if py_file.stat().st_size < 50:
                    continue

                files.append(
                    {
                        "path": str(py_file.absolute()),
                        "name": py_file.stem,
                        "relative_path": str(py_file.relative_to(root)),
                    }
                )

                if len(files) >= max_files:
                    break
            except Exception as e:
                print(f"Error reading {py_file}: {e}", file=sys.stderr)
                continue

    return files


def build_graph_from_files(files: list[dict[str, Any]]) -> dict[str, Any]:
    """Build React Flow graph structure from file list"""
    nodes = []
    edges = []

    for idx, file_info in enumerate(files):
        # Determine node type based on file path
        node_type = "default"
        emoji = "📄"

        if "system" in file_info["name"].lower():
            node_type = "input"
            emoji = "🤖"
        elif "agent" in file_info["name"].lower():
            node_type = "output"
            emoji = "🎬"
        elif "sensor" in file_info["name"].lower():
            emoji = "📡"
        elif "actor" in file_info["name"].lower():
            emoji = "🎭"
        elif "world" in file_info["name"].lower():
            emoji = "🌍"
        elif "robot" in file_info["name"].lower():
            emoji = "🤖"

        # Create label from file name
        label = f"{emoji} {file_info['name']}"

        # Position nodes in a grid
        col = idx % 5
        row = idx // 5

        nodes.append(
            {
                "id": f"file_{idx}",
                "type": node_type,
                "data": {
                    "label": label,
                    "filePath": file_info["path"],
                    "line": 1,
                    "relativePath": file_info["relative_path"],
                },
                "position": {"x": col * 200 + 50, "y": row * 150 + 50},
            }
        )

        # Create some edges between related files
        if idx > 0 and idx % 3 == 0:
            # Connect every 3rd node to previous node
            edges.append(
                {
                    "id": f"e_{idx-1}_{idx}",
                    "source": f"file_{idx-1}",
                    "target": f"file_{idx}",
                    "animated": True,
                }
            )

    return {"nodes": nodes, "edges": edges}


@app.get("/")
async def root():
    return {
        "status": "LK Viewer Backend Running ✅",
        "workspace": WORKSPACE_ROOT,
        "message": "Use /parse to get graph data",
    }


@app.get("/parse")
async def parse_workspace():
    """Find Python files and build graph"""
    if not WORKSPACE_ROOT:
        return {"error": "No workspace root set"}

    print(f"[Backend] Parsing workspace: {WORKSPACE_ROOT}", file=sys.stderr)

    # Find Python files
    files = find_python_files(WORKSPACE_ROOT, max_files=20)

    print(f"[Backend] Found {len(files)} Python files", file=sys.stderr)

    if not files:
        return {
            "error": "No Python files found in lk/ directory",
            "files_count": 0,
            "graph": {"nodes": [], "edges": []},
        }

    # Build graph
    graph = build_graph_from_files(files)

    print(
        f"[Backend] Built graph with {len(graph['nodes'])} nodes and {len(graph['edges'])} edges",
        file=sys.stderr,
    )

    return {"files_count": len(files), "graph": graph, "workspace": WORKSPACE_ROOT}


if __name__ == "__main__":
    if len(sys.argv) > 1:
        WORKSPACE_ROOT = sys.argv[1]
        print(f"[Backend] Starting with workspace: {WORKSPACE_ROOT}", file=sys.stderr)
    else:
        WORKSPACE_ROOT = os.getcwd()
        print(
            f"[Backend] No workspace provided, using: {WORKSPACE_ROOT}", file=sys.stderr
        )

    print("[Backend] Server starting on http://localhost:8765", file=sys.stderr)
    print("[Backend] Try: curl http://localhost:8765/parse", file=sys.stderr)

    # Run server
    uvicorn.run(app, host="127.0.0.1", port=8765, log_level="info")
