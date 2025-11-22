#!/usr/bin/env python3

"""
Simple bandit system with objective function - v5.

Clean architecture demonstrating:
- Self-contained components
- Clean system space (only port wiring)
- Objective function tracking fitness
"""

# BAM
from lk.agent import Agent
from lk.common.graph import Connection
from lk.common.objective_function import RewardObjective
from lk.env import Env

# PYTHON


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("Bandit System with Objective Function - v5")
    print("=" * 70)

    # =========================================================================
    # Create components (self-contained)
    # =========================================================================
    env = Env.from_id("ClassicBandit-v1", n_arms=5, seed=42)
    agent = Agent.from_id("RandomAgent-v1", action_space=env.action_space)
    obj = RewardObjective()  # Self-contained objective

    print(f"\nEnvironment: {env}")
    print(f"Agent: {agent}")
    print(f"Objective: {obj}")

    # =========================================================================
    # System space: ONLY port wiring (clean!)
    # =========================================================================
    reward_connection = Connection(source=env.reward, target=obj.reward_in)

    print("\nPort connections:")
    print(f"  {reward_connection}")

    # =========================================================================
    # Run loop
    # =========================================================================
    num_steps = 1000
    print(f"\nRunning {num_steps} steps...")

    for i in range(num_steps):
        # Execute agent and environment
        action_port = agent(env.obs)
        action = action_port.value  # Get action from port
        obs = env(action)

        # Transfer data through connections
        reward_connection.transfer()

        # Update objective (component controls its execution)
        obj.update()

        # Print progress
        if (i + 1) % 100 == 0:
            fitness = obj.fitness.value
            if fitness:
                print(f"  Step {i + 1}: Fitness = {fitness.value:.2f}")

    # =========================================================================
    # Print final results
    # =========================================================================
    print("\n" + "=" * 70)
    print(obj.summary())
    print("=" * 70 + "\n")


"""
===============================================================================
CUSTOM OBJECTIVE EXAMPLE (Advanced)
===============================================================================

For custom logic, subclass ObjectiveFunction:

```python
from lk.common.objective_function import ObjectiveFunction, ObjectiveResult
from lk.common.port import InputPort, OutputPort
from lk.msgs.msg import Reward

class MyCustomObjective(ObjectiveFunction):
    '''
    Custom objective with complex Python logic.
    All complexity stays INSIDE the component!
    '''
    
    def __init__(self):
        super().__init__(name="custom_objective")
        
        # Define ports
        self.reward_in = InputPort("reward_in", Reward, owner=self)
        self.steps_in = InputPort("steps_in", ScalarValue, owner=self)
        self.fitness = OutputPort("fitness", ObjectiveResult, owner=self)
        
        self._discover_ports()
        
        # Internal state (complex Python ok!)
        self.rewards = []
        self.penalty = 0.0
    
    def update(self):
        '''Complex logic here (spaghetti ok if needed!)'''
        reward_msg = self.reward_in.value
        steps_msg = self.steps_in.value
        
        if reward_msg and steps_msg:
            # Custom logic
            reward = reward_msg.value
            steps = steps_msg.value
            
            self.rewards.append(reward)
            
            # Complex computation
            if len(self.rewards) > 10:
                recent_avg = sum(self.rewards[-10:]) / 10
                if recent_avg < 0.3:
                    self.penalty += 0.1  # Penalize poor performance
            
            fitness = sum(self.rewards) - self.penalty - 0.01 * steps
            
            # Write result
            self.fitness.write(ObjectiveResult(
                fitness=fitness,
                step_count=len(self.rewards),
                components={
                    "reward": sum(self.rewards),
                    "penalty": -self.penalty,
                    "steps": -0.01 * steps
                }
            ))

# Usage (same clean wiring!):
custom_obj = MyCustomObjective()
custom_obj.reward_in.connect(env.reward)
custom_obj.steps_in.connect(env.logs.step_count)

for i in range(1000):
    action = agent(env.obs)
    obs = env(action)
    custom_obj.update()  # Component decides what to do
```

===============================================================================
KEY PRINCIPLES
===============================================================================

1. Components are self-contained
   - All logic inside component
   - Can use complex Python (spaghetti ok!)
   - Component decides when/how to execute

2. System space is clean
   - Only port wiring
   - No configuration/specs in system space
   - Just: component.port.connect(other.port)

3. Like ROS architecture
   - Distributed system
   - Port-based messaging
   - Flexible components

===============================================================================
"""
