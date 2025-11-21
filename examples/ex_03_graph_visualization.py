#!/usr/bin/env python3

"""
Graph Visualization Example

Demonstrates the graph visualization capabilities of like_keras:
- Static Mermaid.js visualization
- Mermaid diagram export
- Interactive Dash Cytoscape viewer

Shows how to visualize system architecture similar to Keras plot_model.
"""

# BAM
from lk.common.component import Component
from lk.common.system import System, SystemConfig
from lk.common.port import InputPort, OutputPort
from lk.msgs.msg import Msg

# PYTHON
from dataclasses import dataclass


# Define some example components for our system
class SensorComponent(Component):
    """Example sensor that produces data."""

    @dataclass
    class Config(Component.Config):
        sensor_id: str = "sensor_0"

    def __init__(self, name: str = "Sensor", config: Config = None):
        # Create ports first
        self.output = OutputPort(msg_type=Msg, name="readings")
        # Then call super to discover and register them
        super().__init__(name=name, config=config)


class ProcessorComponent(Component):
    """Example processor that transforms data."""

    def __init__(self, name: str = "Processor"):
        # Create ports first
        self.input = InputPort(msg_type=Msg, name="raw_data")
        self.output = OutputPort(msg_type=Msg, name="processed_data")
        # Then call super to discover and register them
        super().__init__(name=name)


class FilterComponent(Component):
    """Example filter that cleans data."""

    def __init__(self, name: str = "Filter"):
        # Create ports first
        self.input = InputPort(msg_type=Msg, name="data_in")
        self.output = OutputPort(msg_type=Msg, name="filtered_out")
        # Then call super to discover and register them
        super().__init__(name=name)


class ActuatorComponent(Component):
    """Example actuator that consumes data."""

    def __init__(self, name: str = "Actuator"):
        # Create ports first
        self.input = InputPort(msg_type=Msg, name="commands")
        # Then call super to discover and register them
        super().__init__(name=name)


def create_example_system() -> System:
    """
    Create an example system with multiple connected components.
    """
    # Create components
    sensor1 = SensorComponent(name="FrontSensor")
    sensor2 = SensorComponent(name="BackSensor")
    processor = ProcessorComponent(name="DataProcessor")
    filter_comp = FilterComponent(name="NoiseFilter")
    actuator = ActuatorComponent(name="MotorController")

    # Create system and add components
    system = System(
        components=[sensor1, sensor2, processor, filter_comp, actuator],
        config=System.Config(name="RobotSystem", verbose=True),
    )

    # Connect components to create a dataflow graph
    system.connect(sensor1.output, processor.input)
    system.connect(processor.output, filter_comp.input)
    system.connect(filter_comp.output, actuator.input)
    # Create a second path from sensor2
    system.connect(sensor2.output, filter_comp.input)

    # Configure the system to build the graph
    # (not required for visualization, but ensures graph is fully built)
    system.configure()

    return system


def demo_static_visualization():
    """
    Demonstrate static Mermaid.js visualization.
    """
    print("=" * 60)
    print("Demo 1: Static Mermaid.js Visualization")
    print("=" * 60)

    system = create_example_system()

    # Method 1: Auto-open in browser (like Dora's --open flag)
    print("\n1. Auto-open graph in browser...")
    print("   (Set show=False if you don't want auto-open)")
    output_file = system.plot_graph(
        show=True,  # Auto-opens in your default browser!
        rankdir="LR",  # Left-to-right layout
    )
    print(f"   HTML file: {output_file}")

    # Method 2: Get Mermaid diagram string for mermaid.live
    print("\n2. Copy-paste to https://mermaid.live/ ...")
    mermaid_str = system.to_mermaid(rankdir="TB")  # Top-to-bottom layout
    print("   " + "=" * 50)
    print("   Copy the diagram below and paste it on:")
    print("   https://mermaid.live/")
    print("   " + "=" * 50)
    print(mermaid_str)
    print("   " + "=" * 50)
    print("   ☝️  Copy the diagram above to view it on mermaid.live")

    # Method 3: Save without opening
    print("\n3. Save graph without auto-opening...")
    output_file2 = system.plot_graph(
        to_file="my_custom_graph.html", show=True  # Don't auto-open
    )


def demo_interactive_viewer():
    """
    Demonstrate interactive Dash Cytoscape viewer.

    Note: Requires dash and dash-cytoscape installed:
    pip install dash dash-cytoscape
    """
    print("\n" + "=" * 60)
    print("Demo 2: Interactive Dash Cytoscape Viewer")
    print("=" * 60)

    system = create_example_system()

    print("\nLaunching interactive viewer...")
    print("This will start a web server on http://localhost:8050")
    print("Features:")
    print("  - Zoom and pan")
    print("  - Click components to see details")
    print("  - Change layout algorithms")
    print("  - Interactive graph manipulation")
    print("\nPress Ctrl+C to stop the server")

    try:
        system.launch_viewer(port=8050, debug=False)
    except ImportError as e:
        print(f"\nSkipping interactive viewer: {e}")
        print("Install with: pip install dash dash-cytoscape")


def demo_using_utils_directly():
    """
    Demonstrate using utils functions directly.
    """
    print("\n" + "=" * 60)
    print("Demo 3: Using Utils Functions Directly")
    print("=" * 60)

    system = create_example_system()

    # Import from utils
    from lk.utils import system_to_mermaid, plot_system, system_to_cytoscape_elements

    print("\n1. Using system_to_mermaid()...")
    mermaid = system_to_mermaid(system, rankdir="LR")
    print(f"   Generated {len(mermaid)} characters of Mermaid code")

    print("\n2. Using plot_system()...")
    path = plot_system(system, to_file="direct_plot.html", show=False)
    print(f"   Saved to: {path}")

    print("\n3. Using system_to_cytoscape_elements()...")
    elements = system_to_cytoscape_elements(system)
    print(f"   Generated {len(elements)} Cytoscape elements")
    print(f"   - Nodes: {sum(1 for e in elements if 'source' not in e['data'])}")
    print(f"   - Edges: {sum(1 for e in elements if 'source' in e['data'])}")


if __name__ == "__main__":
    import sys

    # demo_static_visualization()
    demo_interactive_viewer()
    # demo_using_utils_directly()

    print("\n" + "=" * 60)
    print("Examples complete!")
    print("=" * 60)
