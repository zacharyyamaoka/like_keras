#!/usr/bin/env python3
"""
System Tracer - Standalone CLI (No FastAPI!)

Usage: python system_tracer.py <file_path> <class_name> <workspace_root>
Output: JSON to stdout
"""

import sys
import json
import ast
from pathlib import Path
from typing import Any


def trace_system_class(
    file_path: str, class_name: str, workspace_root: str
) -> dict[str, Any]:
    """Trace a System class and extract its components"""

    try:
        with open(file_path, "r") as f:
            tree = ast.parse(f.read(), filename=file_path)
    except Exception as e:
        return {"error": f"Failed to parse file: {e}"}

    # Find the class
    target_class = None
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            target_class = node
            break

    if not target_class:
        return {"error": f"Class {class_name} not found in {file_path}"}

    # Extract components from __init__
    components = []
    for item in target_class.body:
        if isinstance(item, ast.FunctionDef) and item.name == "__init__":
            for stmt in item.body:
                if isinstance(stmt, ast.Assign):
                    for target in stmt.targets:
                        if isinstance(target, ast.Attribute):
                            if (
                                isinstance(target.value, ast.Name)
                                and target.value.id == "self"
                            ):
                                comp_name = target.attr
                                comp_type = "Unknown"

                                # Get type from assignment
                                if isinstance(stmt.value, ast.Call):
                                    if isinstance(stmt.value.func, ast.Name):
                                        comp_type = stmt.value.func.id
                                    elif isinstance(stmt.value.func, ast.Attribute):
                                        comp_type = stmt.value.func.attr

                                components.append(
                                    {
                                        "name": comp_name,
                                        "type": comp_type,
                                        "line": stmt.lineno,
                                    }
                                )

    # Build graph
    nodes = []
    edges = []

    # Root node - the system
    nodes.append(
        {
            "id": "system_root",
            "type": "input",
            "data": {
                "label": f"🤖 {class_name}",
                "filePath": file_path,
                "line": target_class.lineno,
                "type": "System",
            },
            "position": {"x": 250, "y": 50},
        }
    )

    # Component nodes
    for idx, comp in enumerate(components):
        node_id = f"comp_{idx}"

        # Determine emoji
        emoji = "📦"
        node_type = "default"
        comp_type_lower = comp["type"].lower()

        if "robot" in comp_type_lower:
            emoji = "🤖"
        elif "sensor" in comp_type_lower:
            emoji = "📡"
        elif "actor" in comp_type_lower or "actuator" in comp_type_lower:
            emoji = "🎬"
            node_type = "output"
        elif "controller" in comp_type_lower or "control" in comp_type_lower:
            emoji = "⚙️"
        elif "env" in comp_type_lower or "world" in comp_type_lower:
            emoji = "🌍"
        elif "agent" in comp_type_lower:
            emoji = "🎬"
            node_type = "output"

        nodes.append(
            {
                "id": node_id,
                "type": node_type,
                "data": {
                    "label": f'{emoji} {comp["name"]}\n({comp["type"]})',
                    "filePath": file_path,
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

        # Connect to root
        edges.append(
            {
                "id": f"e_root_{node_id}",
                "source": "system_root",
                "target": node_id,
                "animated": True,
                "label": "contains",
            }
        )

    return {
        "success": True,
        "system_class": class_name,
        "file_path": file_path,
        "components_count": len(components),
        "components": components,
        "graph": {"nodes": nodes, "edges": edges},
    }


if __name__ == "__main__":
    if len(sys.argv) < 4:
        result = {
            "error": "Usage: system_tracer.py <file_path> <class_name> <workspace_root>",
            "example": "python system_tracer.py /path/to/system.py MySystem /workspace",
        }
        print(json.dumps(result, indent=2))
        sys.exit(1)

    file_path = sys.argv[1]
    class_name = sys.argv[2]
    workspace_root = sys.argv[3]

    # Trace the system
    result = trace_system_class(file_path, class_name, workspace_root)

    # Output JSON to stdout
    print(json.dumps(result, indent=2))
