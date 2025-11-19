#!/usr/bin/env python3

"""
    Example: Foxglove Backend for Live Visualization
    
    Demonstrates using the Foxglove backend to:
    - Send log messages to Foxglove WebSocket server for live viewing
    - Record messages to MCAP file for later playback
    - Log different message types (strings, Foxglove schemas, custom data)
    - Use topics for message routing
    
    Requirements:
        pip install foxglove-sdk
    
    Usage:
        1. Run this script
        2. Open Foxglove app (https://foxglove.dev)
        3. Connect to ws://localhost:8765
        4. View log messages in the Log panel
        5. After script completes, open the MCAP file in Foxglove
"""

# BAM
from bam.common.logger import Logger

# PYTHON
import time

def main():
    print("=" * 60)
    print("Foxglove Backend Example")
    print("=" * 60)
    print()
    print("Instructions:")
    print("1. Open Foxglove app (https://foxglove.dev)")
    print("2. Connect to ws://localhost:8765")
    print("3. Add a 'Log' panel to view messages")
    print("4. Watch messages appear in real-time!")
    print()
    
    # Create logger with Foxglove backend
    logger = Logger(run_id="foxglove_demo", robot_id="BAM-01")
    
    # Add Python backend for console output
    logger.get_python_backend(level="INFO")
    
    # Add Foxglove backend for live visualization and recording
    logger.get_foxglove_backend(
        server_enabled=True,
        mcap_file="/tmp/bam_logger_demo.mcap",
        server_port=8765,
        verbose=True
    )
    
    print("\n" + "=" * 60)
    print("Logging messages...")
    print("=" * 60 + "\n")
    
    # Log simple string messages
    logger.info("System initialized", component="main")
    logger.debug("Starting calibration", sensor_count=5)
    
    # Log with topics for routing
    logger.info("Arm controller ready", topic="/robot/arm", dof=6)
    logger.info("Gripper controller ready", topic="/robot/gripper", force_n=100)
    
    # Log different levels
    logger.warning("High CPU usage", topic="/system/health", cpu_percent=85)
    logger.error("Sensor timeout", topic="/system/errors", sensor_id="cam_01", timeout_ms=5000)
    
    # Simulate periodic logging
    print("\nSimulating periodic sensor readings...")
    for i in range(5):
        logger.info(
            f"Sensor reading {i}",
            topic="/sensors/readings",
            temperature_c=25.0 + i * 0.5,
            humidity_percent=45 + i,
            throttle_duration_sec=0.5  # Rate limit to 2Hz
        )
        time.sleep(0.3)
    
    # Log with custom timestamps
    import time
    log_time = time.time()
    logger.info(
        "Historical data point",
        topic="/historical",
        log_time=log_time - 3600,  # 1 hour ago
        value=42
    )
    
    # Use Foxglove schemas directly (if needed)
    try:
        from foxglove.schemas import Log, LogLevel, Timestamp
        
        custom_log = Log(
            timestamp=Timestamp.now(),
            level=LogLevel.Info,
            message="Direct Foxglove schema message"
        )
        logger.info(custom_log, topic="/custom/foxglove")
    except ImportError:
        print("Foxglove schemas not available for direct usage demo")
    
    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)
    print()
    print("Next steps:")
    print("- Open the MCAP file: /tmp/bam_logger_demo.mcap")
    print("- Drag and drop into Foxglove app for playback")
    print("- Filter logs by topic, level, or search text")


if __name__ == "__main__":
    try:
        main()
    except ImportError as e:
        print(f"\nError: {e}")
        print("\nMissing dependency. Install with:")
        print("  pip install foxglove-sdk")
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

