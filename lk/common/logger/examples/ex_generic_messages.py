#!/usr/bin/env python3

"""
Example: Generic Message Types

Demonstrates the flexibility of the new generic logger that accepts any message type:
- String messages (most common)
- Dict messages (structured data)
- Dataclass messages (BAM/ROS messages)
- Foxglove schema messages
- Visual objects

Shows how the logger routes different message types to appropriate backends.
"""

# BAM
from bam.common.logger import Logger
from bam.msgs import Pose, Point, Quaternion

# PYTHON
import time
from dataclasses import dataclass


@dataclass
class SensorReading:
    """Example custom dataclass message."""

    sensor_id: str
    temperature_c: float
    humidity_pct: float
    timestamp: float


def main():
    print("=" * 60)
    print("Generic Message Types Example")
    print("=" * 60)
    print()

    # Create logger with Python backend
    logger = Logger(experiment_id="msg_types_demo")
    logger.get_python_backend(level="INFO")

    print("\n1. String Messages (Most Common)")
    print("-" * 60)

    # Standard string logging
    logger.info("System initialized")
    logger.debug("Loading configuration", config_file="/etc/robot.yaml")
    logger.warning("High CPU usage detected", cpu_percent=87.5)

    print("\n2. Dict Messages")
    print("-" * 60)

    # Dict messages are automatically stringified
    status_dict = {
        "status": "operational",
        "uptime_hours": 24.5,
        "errors": 0,
        "warnings": 2,
    }
    logger.info(status_dict, component="health_monitor")

    # Nested dict
    complex_data = {
        "robot": {"id": "BAM-01", "location": {"x": 1.5, "y": 2.3, "z": 0.0}},
        "task": {"name": "pick_and_place", "progress": 0.65},
    }
    logger.info(complex_data, topic="/robot/status")

    print("\n3. Dataclass Messages (BAM/ROS types)")
    print("-" * 60)

    # Custom dataclass
    reading = SensorReading(
        sensor_id="temp_01",
        temperature_c=25.3,
        humidity_pct=45.2,
        timestamp=time.time(),
    )
    logger.info(reading, topic="/sensors/temperature")

    # ROS message types
    pose = Pose(
        position=Point(x=0.5, y=0.3, z=0.2),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    logger.info(pose, topic="/robot/pose")

    print("\n4. Mixed Logging with Topics")
    print("-" * 60)

    # Different topics for organization
    logger.info("Motor controller ready", topic="/hardware/motors")
    logger.info("Camera feed active", topic="/hardware/sensors", fps=30)
    logger.info("Planning completed", topic="/brain/planning", duration_ms=125)

    print("\n5. With Timestamps")
    print("-" * 60)

    # Log with custom timestamp (useful for historical data)
    past_time = time.time() - 3600  # 1 hour ago
    logger.info(
        "Historical sensor reading",
        topic="/sensors/history",
        log_time=past_time,
        value=42.0,
    )

    # Current time (default if not specified)
    logger.info("Current reading", topic="/sensors/live", value=43.2)

    print("\n6. Rate-Limited Logging")
    print("-" * 60)

    print("Logging in fast loop with throttle...")
    for i in range(20):
        logger.info(
            f"Loop iteration {i}",
            topic="/control/loop",
            iteration=i,
            throttle_duration_sec=1.0,  # Max 1 message per second
        )
        time.sleep(0.1)  # 10Hz loop

    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)
    print()
    print("Key takeaways:")
    print("  • msg parameter accepts ANY type")
    print("  • topic parameter for routing/organization")
    print("  • log_time for custom timestamps")
    print("  • Backends handle type conversion automatically")
    print("  • String messages remain super lightweight")


if __name__ == "__main__":
    main()
