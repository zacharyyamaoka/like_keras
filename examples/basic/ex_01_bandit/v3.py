#!/usr/bin/env python3

"""
Simple bandit system with random agent.

Demonstrates the simplest possible reinforcement learning system:
- Agent samples random actions
- Environment provides rewards
- System runs for multiple episodes
"""

# BAM
from lk.agent.random_agent import RandomAgent
from lk.env.gym_adapter_env.v1 import GymAdapterEnv

# PYTHON


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("Simple Bandit System with Random Agent")
    print("=" * 70)

    # Create environment - ClassicBandit is now registered
    env = GymAdapterEnv.from_id("ClassicBandit-v1", n_arms=5, seed=42)
    print(f"\nEnvironment: {env}")
    print(f"Action space: {env.action_space}")

    # Create random agent
    agent = RandomAgent.from_id("RandomAgent-v1")

    # Set the agent's action space to match the environment
    agent.config.action_space = env.action_space
    print(f"Agent: {agent}")

    # Reset the environment to get initial observation
    obs = env.reset()
    print(f"\nInitial observation: {obs.data}")

    # Run 10 episodes
    print("\nRunning 10 episodes:")
    for i in range(10):
        # Agent selects action based on observation
        action = agent.forward(obs)

        # Environment executes action and returns new observation
        obs = env(action)

        # Access reward from the environment's reward port
        reward = env.reward.value

        # Print the reward
        print(f"  Episode {i}: action={action.data}, reward={reward.value}")

    print("\n" + "=" * 70)


"""
Ok great, that is is working. some changes thouhg I want to make in the next version.

1. Please make it so we can from.id on the top level Env and Agent. Just like how in gym you can do the top level registration.

I see how maby doing the lowever level could be helpful as well if you want specific env type hinting...

So maby we can use a name spacing scheme. The children can automatically append their class name as a namespace? that feels a bit complicated tbh.

For now how about we just use unique ids...

So should be env = Env.from_id("ClassicBandit-v1", n_arms=5, seed=42) # this matches the gym.make(), which pass thorugh.. 
and for agent should be Agent.from_id("RandomAgent-v1")

2. The api should better follow the keras on

It should be 

action = agent(env.obs)
obs = env(action)

The env should auto reset when obs is called. 


3. We need an objective function to track progress. A good system here is the one that can maximise reward over a given time horizon. lets say 1000 steps

Please track the reward in a list, and sum it at the end and print out the value
 all thes changes should go into v4.py

Please make sure that the code runs and that we get a list of reward and a final reward at the end
 
"""
