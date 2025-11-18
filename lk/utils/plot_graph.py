#!/usr/bin/env python3

"""
    Graph visualization utilities for System objects.
    
    Provides functions to visualize system architecture using:
    - Mermaid.js for static graph visualization
    - Dash Cytoscape for interactive dynamic graphs
    
    Inspired by Keras model plotting utilities.
"""

# PYTHON
from typing import Optional, TYPE_CHECKING
import tempfile
import webbrowser
from pathlib import Path

if TYPE_CHECKING:
    from lk.common.system import System


MERMAID_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="utf-8">
    <title>{title}</title>
</head>
<body>
    <div class="mermaid">
{mermaid_code}
    </div>
    <script src="https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js"></script>
    <script>
        mermaid.initialize({{ 
            startOnLoad: true, 
            securityLevel: 'loose', 
            theme: 'base',
            flowchart: {{
                useMaxWidth: true,
                htmlLabels: true
            }}
        }});
    </script>
</body>
</html>
"""


def system_to_mermaid(system: 'System', rankdir: str = 'TB') -> str:
    """
        Convert a System to Mermaid.js diagram format.
        
        Args:
            system: The System to visualize
            rankdir: Direction of graph layout ('LR' = left-to-right, 'TB' = top-to-bottom)
            
        Returns:
            Mermaid diagram as string
    """
    lines = []
    lines.append(f"graph {rankdir}")
    
    # Add components as nodes
    component_ids = {}
    for i, component in enumerate(system.components):
        comp_id = f"comp{i}"
        component_ids[component] = comp_id
        
        # Create label with component name and type
        comp_type = component.__class__.__name__
        label = f"{component.name}"
        if comp_type != component.name:
            label = f"{component.name}<br/><i>{comp_type}</i>"
        
        lines.append(f'    {comp_id}["{label}"]')
    
    # Add connections as edges
    for conn in system.graph.connections:
        src_comp = conn.source.owner
        tgt_comp = conn.target.owner
        
        if src_comp and tgt_comp and src_comp in component_ids and tgt_comp in component_ids:
            src_id = component_ids[src_comp]
            tgt_id = component_ids[tgt_comp]
            
            # Create edge label with port names
            edge_label = f"{conn.source.name} → {conn.target.name}"
            lines.append(f'    {src_id} -->|"{edge_label}"| {tgt_id}')
    
    # Add subsystems as subgraphs
    for i, subsystem in enumerate(system._subsystems):
        lines.append(f'    subgraph subsystem{i}["{subsystem.config.name}"]')
        # Recursively add subsystem components (simplified)
        for j, comp in enumerate(subsystem.components):
            sub_comp_id = f"sub{i}_comp{j}"
            label = f"{comp.name}"
            lines.append(f'        {sub_comp_id}["{label}"]')
        lines.append('    end')
    
    return '\n'.join(lines)


def plot_system(system: 'System',
                to_file: Optional[str] = None,
                show: bool = False,
                rankdir: str = 'LR',
                title: Optional[str] = None) -> Optional[str]:
    """
        Visualize a System as a static graph using Mermaid.js.
        
        Similar to Keras's plot_model function.
        
        Args:
            system: The System to visualize
            to_file: Path to save HTML file. If None, auto-generates filename.
            show: Whether to open the visualization in browser
            rankdir: Direction of graph layout ('LR' = left-to-right, 'TB' = top-to-bottom)
            title: Title for the HTML page (defaults to system name)
            
        Returns:
            Path to the generated HTML file
    """
    # Generate mermaid diagram
    mermaid_code = system_to_mermaid(system, rankdir=rankdir)
    
    # Create HTML
    html_title = title or f"System: {system.config.name}"
    html_content = MERMAID_TEMPLATE.format(
        title=html_title,
        mermaid_code=mermaid_code
    )
    
    # Determine output file (similar to Dora's approach)
    if to_file is None:
        # Auto-generate filename, avoiding overwrites
        working_dir = Path.cwd()
        graph_filename = f"{system.config.name}-graph"
        extra = 0
        while True:
            if extra == 0:
                adjusted_filename = f"{graph_filename}.html"
            else:
                adjusted_filename = f"{graph_filename}.{extra}.html"
            output_path = working_dir / adjusted_filename
            if not output_path.exists():
                break
            extra += 1
        output_path = str(output_path)
    else:
        output_path = to_file
    
    # Write HTML file
    with open(output_path, 'w') as f:
        f.write(html_content)
    
    # Print instructions (similar to Dora)
    abs_path = Path(output_path).absolute()
    print(f"View graph by opening the following in your browser:")
    print(f"  file://{abs_path}")
    print(f"\nOr paste the Mermaid diagram on https://mermaid.live/")
    
    # Open in browser if requested
    if show:
        webbrowser.open(f'file://{abs_path}')
    
    return output_path


def system_to_cytoscape_elements(system: 'System') -> list:
    """
        Convert a System to Cytoscape elements format.
        
        Args:
            system: The System to convert
            
        Returns:
            List of Cytoscape elements (nodes and edges)
    """
    from lk.common.component import ComponentLifecycleMixin
    
    elements = []
    
    # Add components as nodes
    for i, component in enumerate(system.components):
        comp_id = f"comp_{id(component)}"
        comp_type = component.__class__.__name__
        
        # Count input and output ports
        num_inputs = len(component.inputs.all_ports())
        num_outputs = len(component.outputs.all_ports())
        
        # Get lifecycle state if component has lifecycle
        lifecycle_state = None
        classes = 'component'
        if isinstance(component, ComponentLifecycleMixin):
            lifecycle_state = component.lifecycle_state.value
            # Add lifecycle state as a CSS class
            classes = f'component lifecycle-{lifecycle_state}'
        
        elements.append({
            'data': {
                'id': comp_id,
                'label': component.name,
                'type': comp_type,
                'num_inputs': num_inputs,
                'num_outputs': num_outputs,
                'lifecycle_state': lifecycle_state
            },
            'classes': classes
        })
    
    # Add connections as edges
    for i, conn in enumerate(system.graph.connections):
        src_comp = conn.source.owner
        tgt_comp = conn.target.owner
        
        if src_comp and tgt_comp:
            edge_id = f"edge_{i}"
            src_id = f"comp_{id(src_comp)}"
            tgt_id = f"comp_{id(tgt_comp)}"
            
            elements.append({
                'data': {
                    'id': edge_id,
                    'source': src_id,
                    'target': tgt_id,
                    'label': f"{conn.source.name}→{conn.target.name}",
                    'source_port': conn.source.name,
                    'target_port': conn.target.name
                },
                'classes': 'connection'
            })
    
    return elements


def launch_interactive_viewer(system: 'System', 
                              port: int = 8050,
                              debug: bool = False,
                              live_update: bool = False,
                              update_interval_ms: int = 1000):
    """
        Launch an interactive Dash Cytoscape viewer for the System.
        
        This creates a web application that visualizes the system graph
        with interactive features like zoom, pan, and real-time updates.
        
        Args:
            system: The System to visualize
            port: Port to run the Dash server on
            debug: Whether to run in debug mode
            live_update: Enable real-time updates of the graph
            update_interval_ms: Update interval in milliseconds (if live_update=True)
    """
    try:
        from dash import Dash, html, dcc, Input, Output, State
        import dash_cytoscape as cyto
    except ImportError:
        raise ImportError(
            "Interactive viewer requires 'dash' and 'dash-cytoscape'. "
            "Install with: pip install dash dash-cytoscape"
        )
    
    # Create Dash app
    app = Dash(__name__)
    
    # Get initial elements
    elements = system_to_cytoscape_elements(system)
    
    # Define stylesheet with lifecycle state colors
    stylesheet = [
        # Base component style
        {
            'selector': '.component',
            'style': {
                'label': 'data(label)',
                'text-valign': 'center',
                'text-halign': 'center',
                'background-color': '#95a5a6',  # Default gray
                'color': '#fff',
                'width': 80,
                'height': 80,
                'shape': 'roundrectangle',
                'font-size': '12px',
                'border-width': 3,
                'border-color': '#7f8c8d'
            }
        },
        # Lifecycle states with distinct colors
        {
            'selector': '.lifecycle-unconfigured',
            'style': {
                'background-color': '#95a5a6',  # Gray - not configured
                'border-color': '#7f8c8d'
            }
        },
        {
            'selector': '.lifecycle-inactive',
            'style': {
                'background-color': '#f39c12',  # Orange - configured but inactive
                'border-color': '#d68910'
            }
        },
        {
            'selector': '.lifecycle-active',
            'style': {
                'background-color': '#2ecc71',  # Green - actively running
                'border-color': '#27ae60'
            }
        },
        {
            'selector': '.lifecycle-finalized',
            'style': {
                'background-color': '#34495e',  # Dark gray - finalized/destroyed
                'border-color': '#2c3e50'
            }
        },
        # Connection edges
        {
            'selector': '.connection',
            'style': {
                'label': 'data(label)',
                'curve-style': 'bezier',
                'target-arrow-shape': 'triangle',
                'arrow-scale': 1.5,
                'line-color': '#95a5a6',
                'target-arrow-color': '#95a5a6',
                'width': 2,
                'font-size': '10px',
                'text-rotation': 'autorotate'
            }
        },
        # Selected elements
        {
            'selector': ':selected',
            'style': {
                'background-color': '#e74c3c',
                'line-color': '#e74c3c',
                'target-arrow-color': '#e74c3c',
                'border-color': '#c0392b'
            }
        }
    ]
    
    # Build layout components with fullscreen graph and bottom panel
    layout_components = [
        # Top header bar - compact
        html.Div([
            html.Div([
                html.H2(f"🔷 {system.config.name}", style={'margin': '0', 'display': 'inline-block', 'color': '#2c3e50'}),
            ], style={'display': 'inline-block', 'marginRight': '30px'}),
            
            # Lifecycle legend - compact
            html.Div([
                html.Span("⬤", style={'color': '#95a5a6', 'fontSize': '16px', 'marginRight': '5px'}),
                html.Span("Unconfigured  ", style={'fontSize': '12px', 'marginRight': '12px'}),
                html.Span("⬤", style={'color': '#f39c12', 'fontSize': '16px', 'marginRight': '5px'}),
                html.Span("Inactive  ", style={'fontSize': '12px', 'marginRight': '12px'}),
                html.Span("⬤", style={'color': '#2ecc71', 'fontSize': '16px', 'marginRight': '5px'}),
                html.Span("Active  ", style={'fontSize': '12px', 'marginRight': '12px'}),
                html.Span("⬤", style={'color': '#34495e', 'fontSize': '16px', 'marginRight': '5px'}),
                html.Span("Finalized", style={'fontSize': '12px'}),
            ], style={'display': 'inline-block', 'marginRight': '30px'}),
            
            # Layout dropdown
            html.Div([
                html.Label("Layout: ", style={'marginRight': '8px'}),
                dcc.Dropdown(
                    id='layout-dropdown',
                    options=[
                        {'label': 'Hierarchical', 'value': 'breadthfirst'},
                        {'label': 'Force-directed', 'value': 'cose'},
                        {'label': 'Circle', 'value': 'circle'},
                        {'label': 'Grid', 'value': 'grid'},
                        {'label': 'Concentric', 'value': 'concentric'}
                    ],
                    value='breadthfirst',
                    style={'width': '180px', 'display': 'inline-block'}
                )
            ], style={'display': 'inline-block'}),
            
        ], style={
            'padding': '10px 20px',
            'backgroundColor': '#ecf0f1',
            'borderBottom': '2px solid #bdc3c7',
            'display': 'flex',
            'alignItems': 'center',
            'justifyContent': 'space-between'
        }),
        
        # Main graph - takes up remaining screen space
        html.Div([
            cyto.Cytoscape(
                id='system-graph',
                elements=elements,
                stylesheet=stylesheet,
                style={'width': '100%', 'height': 'calc(100vh - 250px)'},  # Full height minus header and panel
                layout={'name': 'breadthfirst', 'directed': True}
            ),
        ], style={'position': 'relative'}),
        
        # Bottom info panel - collapsible
        html.Div(id='info-panel-container', children=[
            html.Div(id='selected-info', style={'padding': '15px'})
        ], style={
            'position': 'fixed',
            'bottom': '0',
            'left': '0',
            'right': '0',
            'maxHeight': '200px',
            'overflowY': 'auto',
            'backgroundColor': '#ffffff',
            'borderTop': '3px solid #3498db',
            'boxShadow': '0 -2px 10px rgba(0,0,0,0.1)',
            'transition': 'transform 0.3s ease',
            'transform': 'translateY(100%)',  # Hidden by default
            'zIndex': 1000
        })
    ]
    
    # Add live update interval if enabled
    if live_update:
        layout_components.append(
            dcc.Interval(
                id='update-interval',
                interval=update_interval_ms,
                n_intervals=0
            )
        )
        print(f"⚡ Live updates enabled (every {update_interval_ms}ms)")
    
    # Create layout
    app.layout = html.Div(layout_components)
    
    # Callback to update layout
    @app.callback(
        Output('system-graph', 'layout'),
        Input('layout-dropdown', 'value')
    )
    def update_layout(layout_name):
        return {'name': layout_name, 'directed': True}
    
    # Callback to show/hide info panel and update content
    @app.callback(
        [Output('info-panel-container', 'style'),
         Output('selected-info', 'children')],
        Input('system-graph', 'selectedNodeData')
    )
    def update_info_panel(data_list):
        # Base panel style
        base_style = {
            'position': 'fixed',
            'bottom': '0',
            'left': '0',
            'right': '0',
            'maxHeight': '200px',
            'overflowY': 'auto',
            'backgroundColor': '#ffffff',
            'borderTop': '3px solid #3498db',
            'boxShadow': '0 -2px 10px rgba(0,0,0,0.1)',
            'transition': 'transform 0.3s ease',
            'zIndex': 1000
        }
        
        if not data_list:
            # Hide panel when nothing selected
            base_style['transform'] = 'translateY(100%)'
            return base_style, html.Div()
        
        # Show panel when component selected
        base_style['transform'] = 'translateY(0)'
        
        data = data_list[0]
        
        # Determine lifecycle color
        lifecycle_state = data.get('lifecycle_state')
        if lifecycle_state:
            lifecycle_colors = {
                'unconfigured': '#95a5a6',
                'inactive': '#f39c12',
                'active': '#2ecc71',
                'finalized': '#34495e'
            }
            lifecycle_color = lifecycle_colors.get(lifecycle_state, '#888')
            lifecycle_display = html.Div([
                html.Strong("Lifecycle: ", style={'marginRight': '10px'}),
                html.Span(
                    f"⬤ {lifecycle_state.upper()}",
                    style={'color': lifecycle_color, 'fontWeight': 'bold', 'fontSize': '14px'}
                )
            ], style={'display': 'inline-block', 'marginRight': '30px'})
        else:
            lifecycle_display = html.Div([
                html.Strong("Lifecycle: ", style={'marginRight': '10px'}),
                html.Span("N/A", style={'color': '#888'})
            ], style={'display': 'inline-block', 'marginRight': '30px'})
        
        content = html.Div([
            # Main info row
            html.Div([
                html.Div([
                    html.H3(f"📦 {data.get('label', 'Unknown')}", 
                           style={'margin': '0', 'display': 'inline-block', 'color': '#2c3e50'}),
                    html.Span(f" ({data.get('type', 'Unknown')})", 
                             style={'color': '#7f8c8d', 'fontSize': '14px', 'marginLeft': '10px'})
                ], style={'marginBottom': '10px'}),
                
                # Stats row
                html.Div([
                    lifecycle_display,
                    html.Div([
                        html.Strong("Ports: ", style={'marginRight': '10px'}),
                        html.Span(f"⬇️ {data.get('num_inputs', 0)} inputs", 
                                 style={'marginRight': '15px', 'color': '#3498db'}),
                        html.Span(f"⬆️ {data.get('num_outputs', 0)} outputs", 
                                 style={'color': '#e74c3c'})
                    ], style={'display': 'inline-block'})
                ], style={'display': 'flex', 'alignItems': 'center'}),
            ])
        ], style={'padding': '5px'})
        
        return base_style, content
    
    # Callback for live updates (if enabled)
    if live_update:
        @app.callback(
            Output('system-graph', 'elements'),
            Input('update-interval', 'n_intervals')
        )
        def update_graph(n):
            # Regenerate elements from current system state
            return system_to_cytoscape_elements(system)
    
    print(f"Starting interactive viewer on http://localhost:{port}")
    if live_update:
        print(f"🔴 LIVE mode: Graph will update every {update_interval_ms/1000:.1f}s")
    print("Press Ctrl+C to stop")
    
    # Run the app (Dash 2.x uses app.run instead of app.run_server)
    app.run(debug=debug, port=port)


if __name__ == "__main__":
    """
        Example usage of graph visualization utilities.
    """
    # This example requires the system to be properly set up
    print("Graph visualization utilities")
    print("Usage:")
    print("  from lk.utils.plot_graph import plot_system, system_to_mermaid")
    print("  plot_system(my_system, to_file='system_graph.html', show=True)")
    print("  launch_interactive_viewer(my_system)")

