#!/usr/bin/env python3

"""
How can we describe a cart pole system in as few lines as possible?

Follow gym api naming: EnvironmentName-v[VersionNumber]

Actually to make this a general purpose library we just need to define a really nice system to compose components together...
"""

# BAM
from lk import System, Node, Agent, Env
from lk import Observation, Action, Reward, Done, Info

# PYTHON
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass


"""
Various examples going from simple to more complex with fucntional sytle

- Good default principles, it works without passing in any configuration

"""


def generate_system() -> System:

    node = Node(name="node")

    agent_env = AgentEnv.from_node(node)

    return System(
        nodes=[node],
        components=[agent_env],
        outputs=[agent_env.out.action, agent_env.out.obs],
    )


def generate_system() -> System:

    agent_node = Node(name="agent")
    env_node = Node(name="env")

    agent = Agent.from_node(agent_node)
    env = Env.from_node(env_node)

    action = agent(env.obs)
    obs = env(action)

    return System(
        nodes=[agent_node, env_node], components=[agent, env], outputs=[action, obs]
    )


def generate_system() -> System:

    # Define nodes
    # Allows components to optionally make a new node from this node, to be in their own process
    # Otherwise then this becomes a bit more messy... but mabye thats important?
    sensor_node = Node(name="sensors", allow_splitting=True)
    world_model_node = Node(name="world_model")
    actor_node = Node(name="actor")
    actuators_node = Node(name="actuators", allow_splitting=True)

    # Define components
    sensors = Sensors.from_node(sensor_node)
    world_model = WorldModel.from_node(world_model_node)
    actor = Actor.from_node(actor_node)
    actuators = Actuators.from_node(actuators_node)

    # Show how you can just run dynamically or statically

    state = world_model(sensors.obs, actor.action, actuators.feedback)
    action = actor(state)
    feedback = actuators(action)

    return System(
        nodes=[sensor_node, world_model_node, actor_node, actuators_node],
        components=[sensors, world_model, actor, actuators],
        outputs=[sensors.obs, state, action, feedback],
    )


# config has everything we need to remake a class. Apply that recursively You can basically many an aribtuarily sytem with just


if __name__ == "__main__":

    # You can run without a node... then it just defaults to a single pynode in the current process...
    node = PyNode(name="node")
    system_config = System.Config(name="system")

    agent = Agent.from_config(system_config)
    env = Env.from_config(system_config)

    for i in range(10):
        action = agent(env.obs)
        obs = env(action)

    sensors = Sensors.from_config(system_config)
    world_model = WorldModel.from_config(system_config)
    actor = Actor.from_config(system_config)
    actuators = Actuators.from_config(system_config)

    for i in range(10):
        state = world_model(sensors.obs, actor.action, actuators.feedback)
        action = actor(state)
        feedback = actuators(action)

    from .gym_configs.gym1_config import Gym1Config

    System.from_config(Gym1Config)

    # We can do it in a static graph way.
    system = generate_system()
    system.launch()  # this blocks until ctrl+c is pressed

    system.shutdown()

    # If you really want it to be static you can liekyl just define everything in a system config... its just like a dora dataflow... all the inputs outputs etc..?

    # internally env has a

    # tick()
    # reset()
    # step(action)

    # env.obs should return:
    # if not env.done:
    #     self.step(action)
    # else:
    #     return self.reset()
