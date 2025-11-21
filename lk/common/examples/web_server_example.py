#!/usr/bin/env python3

"""
Example: Web server integration with auto-connected diagnostics.

Shows how to:
1. Use DiagnosticsComponent for components with built-in monitoring
2. Add web server to system
3. Auto-connect to all state/diagnostics ports
4. View diagnostics in browser
"""

# BAM
from lk.common.component import Component
from lk.common.diagnostics_mixin import DiagnosticsComponent
from lk.common.system import System
from lk.common.port import InputPort, OutputPort
from lk.msgs.msg import Msg
from lk.msgs.diagnostics import DiagnosticLevel

# PYTHON
from dataclasses import dataclass
import time


# Example message types
@dataclass
class SensorData(Msg):
    value: float = 0.0


# Example component with diagnostics
class SensorComponent(DiagnosticsComponent):
    """
    Sensor component with built-in diagnostics.

    Automatically has state_port and diagnostics_port.
    """

    def __init__(self, name: str = "sensor", hardware_id: str = "sensor_001", **kwargs):
        super().__init__(name=name, **kwargs)
        self.hardware_id = hardware_id

        # Component-specific ports
        self.data_out = OutputPort(name="data", msg_type=SensorData, owner=self)

    def step(self):
        """Simulate sensor reading."""
        import random

        start_time = time.time()

        # Simulate work
        time.sleep(0.01)

        # Generate data
        value = random.uniform(0, 100)
        self.data_out.write(SensorData(value=value))

        # Record execution time
        duration = time.time() - start_time
        self._record_execution_time(duration)

        # Simulate occasional errors
        if random.random() < 0.1:  # 10% chance of error
            self.set_diagnostic_error(
                message="Sensor reading failed", hardware_id=self.hardware_id
            )
        else:
            self.set_diagnostic_ok("Sensor operating normally")


class ProcessorComponent(DiagnosticsComponent):
    """
    Processor component that depends on sensor.
    """

    def __init__(self, name: str = "processor", **kwargs):
        super().__init__(name=name, **kwargs)

        self.data_in = InputPort(name="data", msg_type=SensorData, owner=self)
        self.result_out = OutputPort(name="result", msg_type=SensorData, owner=self)

    def step(self):
        """Process sensor data."""
        start_time = time.time()

        data = self.data_in.read()
        if data:
            # Process data
            processed = SensorData(value=data.value * 2)
            self.result_out.write(processed)

        duration = time.time() - start_time
        self._record_execution_time(duration)
        self.set_diagnostic_ok("Processing complete")


def create_system_with_monitoring():
    """Create a system with web server monitoring."""

    # Create components
    sensor = SensorComponent(hardware_id="urdf_link_arm_sensor")
    processor = ProcessorComponent()

    # Create system
    system = System(
        components=[sensor, processor],
        config=System.Config(name="monitored_system", verbose=True),
    )

    # Connect components
    system.connect(sensor.data_out, processor.data_in)

    # Add web server (auto-connects to all state/diagnostics ports)
    web_server = system.add_web_server(auto_connect=True)

    print(f"\n🌐 Web server starting at http://localhost:{web_server.config.port}")
    print("   Open in browser to see real-time diagnostics!")
    print("   Components with errors will show in red.")
    print("   URDF links will be highlighted when hardware_id is set.\n")

    return system


if __name__ == "__main__":
    system = create_system_with_monitoring()

    # Configure and activate
    system.configure()
    system.activate()

    # Run for a bit
    try:
        for i in range(100):
            system.step()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")

    system.shutdown()
