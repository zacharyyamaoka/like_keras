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



def generate_system() -> System:

    node = Node(
        name="cart_pole",
        component_configs={
            "env": Env.Config(type="CartPole-v1"),
        }
    )

    agent = Agent.from_node(node)
    env = Env.from_node(node)
    action = agent(env.obs)
    obs = env(action)

    return System(nodes=[node], components=[agent, env], inputs=[], outputs=[action, obs], subsystems=[], config=System.Config())


if __name__ == "__main__":
    system = generate_system()
    system.run()