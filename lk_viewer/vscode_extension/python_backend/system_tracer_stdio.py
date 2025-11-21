#!/usr/bin/env python3
"""
System Tracer - Stdio Mode
Long-running process that reads commands from stdin and writes results to stdout.

Usage: python system_tracer_stdio.py --stdio
"""

import sys
import json
import ast
from pathlib import Path
from typing import Any


def trace_system(
    file_path: str, class_name: str, workspace_root: str
) -> dict[str, Any]:
    """Trace a System class and return its component graph"""

    try:
        with open(file_path, "r") as f:
            tree = ast.parse(f.read(), filename=file_path)
    except Exception as e:
        return {"error": f"Failed to parse file: {e}", "success": False}

    # Find the target class
    target_class = None
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            target_class = node
            break

    if not target_class:
        return {"error": f"Class {class_name} not found", "success": False}

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
    nodes = [
        {
            "id": "system_root",
            "type": "input",
            "data": {
                "label": f"🤖 {class_name}",
                "filePath": file_path,
                "line": target_class.lineno,
            },
            "position": {"x": 250, "y": 50},
        }
    ]

    edges = []

    for idx, comp in enumerate(components):
        node_id = f"comp_{idx}"
        emoji = get_emoji_for_type(comp["type"])

        nodes.append(
            {
                "id": node_id,
                "type": (
                    "default" if "actuator" not in comp["type"].lower() else "output"
                ),
                "data": {
                    "label": f'{emoji} {comp["name"]}\n({comp["type"]})',
                    "filePath": file_path,
                    "line": comp["line"],
                },
                "position": {
                    "x": (idx % 4) * 200 + 50,
                    "y": ((idx // 4) + 1) * 150 + 50,
                },
            }
        )

        edges.append(
            {
                "id": f"e_root_{node_id}",
                "source": "system_root",
                "target": node_id,
                "animated": True,
            }
        )

    return {
        "success": True,
        "system_class": class_name,
        "file_path": file_path,
        "components_count": len(components),
        "graph": {"nodes": nodes, "edges": edges},
    }


def get_emoji_for_type(comp_type: str) -> str:
    """Return emoji based on component type"""
    t = comp_type.lower()
    if "robot" in t:
        return "🤖"
    if "sensor" in t:
        return "📡"
    if "actor" in t or "actuator" in t:
        return "🎬"
    if "controller" in t or "control" in t:
        return "⚙️"
    if "env" in t or "world" in t:
        return "🌍"
    if "agent" in t:
        return "🎬"
    return "📦"


def process_command(command: dict[str, Any]) -> dict[str, Any]:
    """Process a single command and return result"""

    cmd_type = command.get("command")

    if cmd_type == "trace_system":
        result = trace_system(
            command["file_path"],
            command["class_name"],
            command.get("workspace_root", ""),
        )
        result["id"] = command.get("id")  # Echo back request ID
        return result

    elif cmd_type == "ping":
        return {"success": True, "message": "pong", "id": command.get("id")}

    else:
        return {
            "success": False,
            "error": f"Unknown command: {cmd_type}",
            "id": command.get("id"),
        }


def main_stdio():
    """Main loop for stdio mode - read commands, write results"""

    print(json.dumps({"status": "ready"}), flush=True)

    for line in sys.stdin:
        try:
            line = line.strip()
            if not line:
                continue

            # Parse command
            command = json.loads(line)

            # Process and return result
            result = process_command(command)

            # Write result as single JSON line
            print(json.dumps(result), flush=True)

        except json.JSONDecodeError as e:
            error_result = {"success": False, "error": f"Invalid JSON: {e}"}
            print(json.dumps(error_result), flush=True)

        except Exception as e:
            error_result = {"success": False, "error": f"Unexpected error: {e}"}
            print(json.dumps(error_result), flush=True)


if __name__ == "__main__":
    if "--stdio" in sys.argv:
        main_stdio()
    else:
        print("Usage: python system_tracer_stdio.py --stdio")
        print("Then send JSON commands via stdin, receive results via stdout")
        sys.exit(1)
