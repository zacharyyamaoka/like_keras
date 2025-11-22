#!/usr/bin/env python3

"""
    Generic Dora node backend.
    
    This module runs as a Dora node and executes like_keras components.
    It's instantiated by the Dora runtime for each node in the dataflow.
"""

# PYTHON
import sys
import pickle
from pathlib import Path


def main(node_id: str):
    """
    Entry point for Dora nodes.
    
    This function is called by Dora with the node ID as argument.
    It loads the components assigned to this node and runs the Dora event loop.
    
    Args:
        node_id: Node identifier from dataflow
    """
    try:
        import dora
    except ImportError:
        print("Error: dora-rs not installed. Install with: pip install dora-rs")
        sys.exit(1)
    
    print(f"Starting Dora node: {node_id}")
    
    # Load components for this node
    components = load_components_for_node(node_id)
    
    if not components:
        print(f"Warning: No components found for node {node_id}")
        return
    
    print(f"Loaded {len(components)} components:")
    for comp in components:
        print(f"  - {comp.name}")
    
    # Create Dora node
    dora_node = dora.Node()
    
    # Main event loop
    print(f"Node {node_id} ready, waiting for events...")
    
    for event in dora_node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            # Handle input event
            input_id = event["id"]
            value = event["value"]
            
            # Find which component this input belongs to
            component, port_name = find_component_for_input(input_id, components)
            
            if component is None:
                print(f"Warning: No component found for input {input_id}")
                continue
            
            # Execute component
            try:
                # Convert Arrow array to Python object if needed
                input_data = arrow_to_python(value)
                
                # Call component forward
                output = component.forward(input_data)
                
                # Send output(s) if any
                if output is not None:
                    output_id = f"{component.name}/output"
                    output_arrow = python_to_arrow(output)
                    dora_node.send_output(output_id, output_arrow)
                    
            except Exception as e:
                print(f"Error executing {component.name}: {e}")
                import traceback
                traceback.print_exc()
        
        elif event_type == "STOP":
            print(f"Node {node_id} received STOP signal")
            break
    
    print(f"Node {node_id} shutting down")


def load_components_for_node(node_id: str):
    """
    Load components assigned to a node.
    
    This is a placeholder - actual implementation will need to:
    1. Load node configuration from build artifacts
    2. Instantiate components
    3. Return list of component instances
    
    Args:
        node_id: Node identifier
        
    Returns:
        List of Component instances
    """
    # TODO: Implement proper component loading
    # For now, return empty list
    # In full implementation:
    # 1. Read .lk_build/dora/node_config_{node_id}.pkl
    # 2. Deserialize component classes and configs
    # 3. Instantiate components
    # 4. Return list
    
    print(f"TODO: Implement component loading for node {node_id}")
    return []


def find_component_for_input(input_id: str, components):
    """
    Find which component an input belongs to.
    
    Args:
        input_id: Input identifier (format: "component_name/port_name")
        components: List of components
        
    Returns:
        Tuple of (component, port_name) or (None, None)
    """
    if '/' not in input_id:
        return None, None
    
    comp_name, port_name = input_id.split('/', 1)
    
    for comp in components:
        if comp.name == comp_name:
            return comp, port_name
    
    return None, None


def arrow_to_python(arrow_array):
    """
    Convert Arrow array to Python object.
    
    Args:
        arrow_array: PyArrow array
        
    Returns:
        Python object
    """
    try:
        import pyarrow as pa
        # Convert to Python list/dict
        return arrow_array.to_pylist()
    except ImportError:
        # Fall back to returning as-is
        return arrow_array


def python_to_arrow(python_obj):
    """
    Convert Python object to Arrow array.
    
    Args:
        python_obj: Python object
        
    Returns:
        PyArrow array
    """
    try:
        import pyarrow as pa
        import numpy as np
        
        # Handle different types
        if isinstance(python_obj, np.ndarray):
            return pa.array(python_obj.tolist())
        elif isinstance(python_obj, (list, tuple)):
            return pa.array(python_obj)
        elif isinstance(python_obj, dict):
            return pa.array([python_obj])
        else:
            # Try to convert to array
            return pa.array([python_obj])
    except Exception as e:
        print(f"Warning: Could not convert to Arrow: {e}")
        return python_obj


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python -m lk.engines.backends.dora_node <node_id>")
        sys.exit(1)
    
    node_id = sys.argv[1]
    main(node_id)

