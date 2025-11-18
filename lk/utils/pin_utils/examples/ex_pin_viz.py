#!/usr/bin/env python3

"""
    Example demonstrating PinViz wrapper usage
    
    Shows how to:
    - Use PinViz with different backends (Viser/Meshcat)
    - Display robot at configurations
    - Toggle collision/visual models
    - Display frames
    - Play trajectories
    - Disable visualization via environment variable
"""

# BAM
from bam.utils.pin_utils import PinViz, PinRobotModel

# PYTHON
import numpy as np
import pinocchio as pin
from robot_descriptions.loaders.pinocchio import load_robot_description

if __name__ == "__main__":
    
    # Load a robot description
    robot_wrapper = load_robot_description("upkie_description")

    robot_model = PinRobotModel(robot_wrapper)
    
    # Create visualizer with Viser backend (default)
    viz = PinViz(robot_model, backend="viser")
    
    # Display at neutral configuration
    q = pin.neutral(viz.model)
    viz.display(q)
    
    print("\n[INFO] Robot displayed at neutral configuration")
    print("[INFO] Viser automatically adds robot control sliders")
    
    # Toggle collision/visual models
    # viz.displayCollisions(True)
    # viz.displayVisuals(False)
    
    # Display frames (if you want to see coordinate frames)
    # viz.displayFrames(True, axis_length=0.2, axis_width=3)
    
    # Play a simple trajectory (oscillate one joint)
    print("\n[INFO] Playing a simple trajectory...")
    n_steps = 50
    q_traj = []
    for i in range(n_steps):
        q_i = q.copy()
        # Oscillate the first joint
        q_i[0] = np.sin(2 * np.pi * i / n_steps) * 0.5
        q_traj.append(q_i)
    
    # Play the trajectory at 30 FPS
    # viz.play(q_traj, dt=1.0/30.0)
    
    print("\n[INFO] Use the Viser interface to control joints interactively")
    print("[INFO] Close the browser window or press Ctrl+C to exit")
    
    # Block to keep the visualization alive
    input("\nPress Enter to exit...")
    
    # --- Alternative: Use Meshcat backend ---
    # viz_meshcat = PinViz(robot_wrapper, backend="meshcat")
    # viz_meshcat.display(q)
    
    # --- Alternative: Disable visualization ---
    # import os
    # os.environ["DISABLE_PIN_VIZ"] = "1"
    # viz_disabled = PinViz(robot_wrapper, backend="viser")
    # viz_disabled.display(q)  # This will be a no-op

