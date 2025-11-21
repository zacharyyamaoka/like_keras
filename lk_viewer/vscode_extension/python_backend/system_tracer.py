#!/usr/bin/env python3
"""
System Tracer - Parses Python System class and generates graph

Traces system composition by:
1. Finding the class definition
2. Parsing __init__ to find components
3. Following component references
4. Building graph of system architecture
"""

# PYTHON
import ast
import inspect
import importlib.util
from pathlib import Path
from typing import Any
from fastapi import FastAPI, Body
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import sys

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

WORKSPACE_ROOT = None


class SystemTraceRequest(BaseModel):
    filePath: str
    className: str
    line: int


class SystemTracer:
    """Trace a System class to build its component graph"""

    def __init__(self, workspace_root: str):
        self.workspace_root = Path(workspace_root)

    def load_module_from_file(self, file_path: str):
        """Dynamically load Python module"""
        try:
            spec = importlib.util.spec_from_file_location("dynamic_module", file_path)
            if spec and spec.loader:
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                return module
        except Exception as e:
            print(f"Error loading module: {e}", file=sys.stderr)
            return None

    def parse_class_ast(self, file_path: str, class_name: str) -> dict[str, Any]:
        """Parse class using AST to find components"""
        with open(file_path, "r") as f:
            tree = ast.parse(f.read(), filename=file_path)

        # Find the class definition
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == class_name:
                return self._extract_components_from_class(node, file_path)

        return None

    def _extract_components_from_class(
        self, class_node: ast.ClassDef, file_path: str
    ) -> dict[str, Any]:
        """Extract component info from class AST node"""
        components = []
        connections = []

        # Look for __init__ method
        for item in class_node.body:
            if isinstance(item, ast.FunctionDef) and item.name == "__init__":
                # Parse __init__ body for self.component = SomeClass()
                for stmt in item.body:
                    if isinstance(stmt, ast.Assign):
                        for target in stmt.targets:
                            if isinstance(target, ast.Attribute):
                                if (
                                    isinstance(target.value, ast.Name)
                                    and target.value.id == "self"
                                ):
                                    component_name = target.attr

                                    # Try to get the type
                                    component_type = self._get_type_from_value(
                                        stmt.value
                                    )

                                    components.append(
                                        {
                                            "name": component_name,
                                            "type": component_type,
                                            "line": stmt.lineno,
                                        }
                                    )

        return {
            "class_name": class_node.name,
            "file_path": file_path,
            "line": class_node.lineno,
            "components": components,
        }

    def _get_type_from_value(self, value_node) -> str:
        """Extract type from assignment value"""
        if isinstance(value_node, ast.Call):
            if isinstance(value_node.func, ast.Name):
                return value_node.func.id
            elif isinstance(value_node.func, ast.Attribute):
                return value_node.func.attr
        return "Unknown"

    def build_system_graph(self, system_info: dict[str, Any]) -> dict[str, Any]:
        """Build React Flow graph from system trace"""
        nodes = []
        edges = []

        # Root node - the system itself
        nodes.append(
            {
                "id": "system_root",
                "type": "input",
                "data": {
                    "label": f"🤖 {system_info['class_name']}",
                    "filePath": system_info["file_path"],
                    "line": system_info["line"],
                    "type": "System",
                },
                "position": {"x": 250, "y": 50},
            }
        )

        # Component nodes
        for idx, comp in enumerate(system_info["components"]):
            node_id = f"comp_{idx}"

            # Determine emoji based on type
            emoji = "📦"
            node_type = "default"

            comp_type_lower = comp["type"].lower()
            if "robot" in comp_type_lower:
                emoji = "🤖"
                node_type = "default"
            elif "sensor" in comp_type_lower:
                emoji = "📡"
            elif "actuator" in comp_type_lower or "actor" in comp_type_lower:
                emoji = "🎬"
                node_type = "output"
            elif "controller" in comp_type_lower:
                emoji = "⚙️"
            elif "env" in comp_type_lower or "world" in comp_type_lower:
                emoji = "🌍"

            nodes.append(
                {
                    "id": node_id,
                    "type": node_type,
                    "data": {
                        "label": f"{emoji} {comp['name']}\n({comp['type']})",
                        "filePath": system_info["file_path"],
                        "line": comp["line"],
                        "componentName": comp["name"],
                        "componentType": comp["type"],
                    },
                    "position": {
                        "x": (idx % 4) * 200 + 50,
                        "y": ((idx // 4) + 1) * 150 + 50,
                    },
                }
            )

            # Connect to system root
            edges.append(
                {
                    "id": f"e_root_{node_id}",
                    "source": "system_root",
                    "target": node_id,
                    "animated": True,
                    "label": "contains",
                }
            )

        return {"nodes": nodes, "edges": edges}

    def trace_system(self, file_path: str, class_name: str) -> dict[str, Any]:
        """Main entry point - trace a system class"""
        print(f"[Tracer] Tracing {class_name} from {file_path}", file=sys.stderr)

        # Parse the class
        system_info = self.parse_class_ast(file_path, class_name)

        if not system_info:
            return {
                "error": f"Class {class_name} not found in {file_path}",
                "graph": {"nodes": [], "edges": []},
            }

        print(
            f"[Tracer] Found {len(system_info['components'])} components",
            file=sys.stderr,
        )

        # Build graph
        graph = self.build_system_graph(system_info)

        return {
            "success": True,
            "system_class": class_name,
            "file_path": file_path,
            "components_count": len(system_info["components"]),
            "graph": graph,
        }


@app.post("/trace_system")
async def trace_system_endpoint(request: SystemTraceRequest):
    """Trace a system class and return its component graph"""

    if not WORKSPACE_ROOT:
        return {"error": "No workspace root set"}

    tracer = SystemTracer(WORKSPACE_ROOT)
    result = tracer.trace_system(request.filePath, request.className)

    return result


@app.get("/")
async def root():
    return {
        "status": "System Tracer Running ✅",
        "workspace": WORKSPACE_ROOT,
        "endpoints": ["/trace_system"],
    }


if __name__ == "__main__":
    if len(sys.argv) > 1:
        WORKSPACE_ROOT = sys.argv[1]
    else:
        WORKSPACE_ROOT = "/home/bam/bam_ws/src/like_keras"

    print(f"[Tracer] Starting with workspace: {WORKSPACE_ROOT}", file=sys.stderr)
    print(f"[Tracer] Server on http://localhost:8765", file=sys.stderr)

    uvicorn.run(app, host="127.0.0.1", port=8765, log_level="info")
