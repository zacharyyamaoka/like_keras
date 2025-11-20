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
    
    return System(nodes=[node], components=[agent_env], outputs=[agent_env.out.action, agent_env.out.obs])

def generate_system() -> System:

    agent_node = Node(name="agent")
    env_node = Node(name="env")

    agent = Agent.from_node(agent_node)
    env = Env.from_node(env_node)
    
    action = agent(env.obs)
    obs = env(action)

    return System(nodes=[agent_node, env_node], components=[agent, env], outputs=[action, obs])


def generate_system() -> System:

    # Define nodes
    # Allows components to optionally make a new node from this node, to be in their own process
    # Otherwise then this becomes a bit more messy... but mabye thats important? 
    sensor_node = Node(name="sensors", allow_splitting=True)
    world_model_node = Node(name="world_model")
    actor_node = Node(name="actor")
    actuators_node = Node(name="actuators")

    # Define components
    sensors = Sensors.from_node(sensor_node)
    world_model = WorldModel.from_node(world_model_node)
    actor = Actor.from_node(actor_node)
    actuators = Actuators.from_node(actuators_node)

    # Assigning essnetial does connecting
    world_model.in.obs = sensors.out.obs
    world_model.in.action = actor.out.action
    world_model.in.feedback = actuators.out.feedback

    actor.in.state = world_model.out.state

    actuators.in.action = actor.out.action

    # connect api (I like how this is readable, perhaps you want to override a connection, etc)
    world_model.in.obs.connect(sensors.out.obs)
    world_model.in.action.connect(actor.out.action)
    world_model.in.feedback.connect(actuators.out.feedback)

    actor.in.state.connect(world_model.out.state)

    actuators.in.action.connect(actor.out.action)

    # Very clean and readable...
    state = world_model(sensors.obs, actor.action, actuators.feedback)
    action = actor(state)
    feedback = actuators(action)


    return System(nodes=[agent_node, env_node], components=[sensors, world_model, actor, actuators], outputs=[sensors.obs, state, action, feedback])


def generate_system() -> System:

    # Sensors and actuators may be running at very different hz...

    # Sensor Nodes
    rs_top_node = Node(name="rs_top")
    rs_left_node = Node(name="rs_left")
    rs_right_node = Node(name="rs_right")
    spectral_node = Node(name="spectral")
    loadcell_node = Node(name="loadcell")


    actor_node = Node(name="actor")
    world_model_node = Node(name="world_model")

    # Actuator Nodes
    moteus_node = Node(name="moteus")
    lights_node = Node(name="lights")
    gui_node = Node(name="gui")

    # Bagging Node
    bagging_node = Node(name="bagging") # hooks into all the different topics and saves into bag...
    foxglove_node = Node(name="foxglove") # hooks into all the different topics and publishes to foxglove

    rs_top = Realsense.from_node(rs_top_node)
    rs_left = Realsense.from_node(rs_left_node)
    rs_right = Realsense.from_node(rs_right_node)
    spectral = Spectral.from_node(spectral_node)
    loadcell = Loadcell.from_node(loadcell_node)
    moteus = Moteus.from_node(moteus_node)
    lights = Lights.from_node(lights_node)

    agent = Agent.from_node(node)
    env = Env.from_node(node)
    
    action = agent(env.obs)
    obs = env(action)

    return System(nodes=[node], components=[agent, env], inputs=[], outputs=[action, obs], subsystems=[], config=System.Config())

# def generate_system() -> System:

#     node = Node(name="node")

#     agent = Agent.from_node(node)
#     env = Env.from_node(node)
    
#     action = agent(env.obs)
#     obs = env(action)

#     return System(nodes=[node], components=[agent, env], inputs=[], outputs=[action, obs], subsystems=[], config=System.Config())


# Multi robot system with 3 robots sharing the same object detection network, and the same grasping model which are running in different processes on an external computer..


# Multi row system with two rows of robots, all using the same obj deteciton entwork and grasping model. with two conveyor belts (MRF in the box)


# Let me describe the actual systems I am trying to build now.. assuming good defaults :) this is a very practical example...


if __name__ == "__main__":
    system = generate_system()
    system.launch() # this blocks until ctrl+c is pressed

    system.shutdown()
