#!/usr/bin/env python3

"""
Simple bandit system with random agent - v4.

Improvements:
1. Top-level Env.from_id() and Agent.from_id() registration
2. Keras-style API: action = agent(env.obs)
3. Objective function: maximize reward over 1000 steps
"""

# BAM

# PYTHON
# Register environments and agents before using them
from lk.agent import Agent
from lk.env import Env

if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("Simple Bandit System with Random Agent - v4")
    print("=" * 70)

    # Environments and agents are auto-registered on import
    # Create environment using top-level Env.from_id()
    env = Env.from_id("ClassicBandit-v1", n_arms=5, seed=42)
    print(f"\nEnvironment: {env}")
    print(f"Action space: {env.action_space}")

    # Create agent using top-level Agent.from_id()
    agent = Agent.from_id("RandomAgent-v1", action_space=env.action_space)
    print(f"Agent: {agent}")

    # Objective: Maximize reward over 1000 steps
    rewards = []
    num_steps = 1000

    print(f"\nRunning {num_steps} steps:")

    # Environment auto-resets on first call and when episodes end
    for i in range(num_steps):
        # Keras-style API: agent takes observation port directly
        action_port = agent(env.obs)
        action = action_port.value

        # Environment executes action (auto-resets when done)
        obs = env(action)

        # Track reward
        reward_value = env.reward.value.value
        rewards.append(reward_value)

        # Print progress every 100 steps
        if (i + 1) % 100 == 0:
            recent_avg = sum(rewards[-100:]) / 100
            print(f"  Step {i + 1}: Recent 100-step avg reward: {recent_avg:.3f}")

    # Print final results
    total_reward = sum(rewards)
    avg_reward = total_reward / num_steps
    print(f"\n{'=' * 70}")
    print("Objective Function Results:")
    print(f"  Total reward over {num_steps} steps: {total_reward:.2f}")
    print(f"  Average reward per step: {avg_reward:.3f}")
    print(f"  Success rate: {avg_reward * 100:.1f}%")
    print(f"{'=' * 70}\n")


"""
Ok in v5 we now want to make this much much more streamlined. compress all the info!

env = Env.from_id("ClassicBandit-v1", n_arms=5, seed=42)
agent = Agent.from_id("RandomAgent-v1", action_space=env.action_space)

# everytime we "launch" a system we do need to spin it for some amount of time...
# Here we use a for loop...
num_steps = 1000
 for i in range(num_steps):

    action = agent(env.obs) # this should directly output an action msg we can pass, shouldn't need to unpack
    obs = env(action)

    # Who should track the reward? 
    # lets just assume its gonna be the agent.
    # So agent is a component, that should have diagonistics that can be accessed via
    agent.diagonistics or agent.dx (for short)

    # for reward I think it makes sense to use a box chart threshold.

    # if we just append the reward to it, then it should autocalculate the mean 

    @dataclass
    class BoxChartThresholds:
        Thresholds for BoxChart statistics (min, max, mean, std).

        min: DiagnosticValue = field(default_factory=DiagnosticValue)
        max: DiagnosticValue = field(default_factory=DiagnosticValue)
        mean_min: DiagnosticValue = field(default_factory=DiagnosticValue)
        mean_max: DiagnosticValue = field(default_factory=DiagnosticValue)
        std: DiagnosticValue = field(default_factory=DiagnosticValue)

        # Track reward
        reward_value = env.reward.value.value
        rewards.append(reward_value)

        # Print progress every 100 steps
        if (i + 1) % 100 == 0:
            recent_avg = sum(rewards[-100:]) / 100
            print(f"  Step {i + 1}: Recent 100-step avg reward: {recent_avg:.3f}")

    ok ok... there are so many potetial places we could read the reward from. What makes most sense? what will be easiet?

    Ok one idea is the the ObjectFunctino is just yet another component, that can list to ports on the different components?

    - You could "stick" your tenecales inside any object and directly access the values, but this doesn't seem very clean and may not scale
    very well...

    just like data recorder is a component. I think it could make sense for objectiveFn To be a compomnent, that eseentially listens to the messages coming from 
    the other components. It should be way more pythonic to just able able to do this... Well I agree... can we make it feel like python?


    So a component as inputs/ouputs logs/diagonistics. I think that it can automatically create output ports for all of its logs/diagonistics. 
    How we actually implement thoose will change. but conceptually, if the component is a completely self contained unit, then we should be explicit 
    about everything that comes in and out. To keep it clean, we don't nessarily need to define all the logs/digainstic output ports as we know
    that they will alwayus be outputs, and that they will always be read at least by the global log aggregator/diagonistics aggregator etc.

    With this sytem it does give us then complex flexibility to tape into any of these ports at any time. This is a very similar design to how do it in ROS...

    Ok so I want to calculate an objective function and I want to add pruning hooks based on the various data in the components. How should this be done in a discpilined way..

    Ok its a mtter then of getting the data. I feel that should be done with ports. Thats interesting very much like Keras. so even if its just symbolic we can make the entire
    graph still beacuse the ports can stub the data... type thing. Ok I think that makes sense.

    Let me get your thoughts though


    ---

    References:
    https://github.com/roboflow/inference/blob/main/inference/core/workflows/execution_engine/v1/compiler/graph_traversal.py
    https://github.com/keras-team/keras/blob/v3.12.0/keras/src/models/model.py#L36
https://github.com/keras-team/keras/blob/v3.12.0/keras/src/layers/layer.py#L69
    https://github.com/keras-team/keras/blob/master/keras/src/ops/operation.py#L18


    ---

    A key thing different from these compile graphs is that its a distributed arhciteceture.

    You cannot compile it in the same way you can a nueral network which is centrally orchestrated run in a single pass...

    
    Your instinct is correct! The graph structure is already there in the ports. You just need lightweight analysis tools on top, not a complete graph execution engine.
    Your system is more like ROS than Keras, and that's exactly right for robotics/RL! 


    ----

    I want the API to be as follows

    obj_fn = ObjectiveFn()

    obj_fn((0.85*env.obs.reward + 0.1*env.)

    # ok one reason this is complicated is that if thoose values are pipe in async you are going to have issues!

    # Isn't the reward basically exactly the object function we care about? We just want to maximise the reward...

    # surely it would make sense that we use that as the out loop function as well?... well the thing is that there are other things we want 
    want to consder in the outloop objective function, like lines of code, etc. 
    Ok so I guess its fine. In most cases I can just have it as a littl epiece of code that runs in the same process as the env

    # ok you basically want to access a particular scalar value.. what if it doesn't exist yet... It hink thats ok
    The msg should support that value if its its not there.

    100* I want to be able to do something like that...

    For this to feel like python. I should be able to connect to any specific value within any msg in the entire system.

    The value of an obejctive function changes over time... so a question is

    do we just provide the current value already summed to the obj fucntion? or do we ask for it to integrated it it self...

    I mean reward is such a common objective... maybe we utlimately write a custom obejct for each one?

    Wel ok to start lets just have a sinmple reward obejctive...

    Generally an obejctive function a good one 

    At the end of the day, we need a fitness score to assign each permutation to sort them. That could be for example
    total reward accumilateed. or avg. reward per episode/step, etc..

    ----

    The key thing is our graph here is at the component level. We are graphing a distirbuted system with poetialyl sync/async 
    communication.

    Within each of thoose components you can do what ever you want intnetionally! make the most complicated python spagheti code you can imagine.

    In that sense the objective functino is just a component container, all the implementation should definetly be inside!

    We definetly don't want to be doings things like construction it from ports on the outside in "system" space, system space should be very clean with just wires for the ports.

    So something Like this I don't think is that nice:

    # Define objective function as a component
    obj_fn = ObjectiveFunction.from_spec(
        metric="total_reward",
        sources=[env.reward],  # Subscribe to reward port
        aggregation="sum",
        window=1000
    )

    # Or custom objective combining multiple sources
    obj_fn = ObjectiveFunction.from_spec(
        metric="composite_score",
        sources={
            "reward": env.reward,
            "code_complexity": agent.diagnostics.loc,
        },
        weights={"reward": 0.85, "code_complexity": 0.15},
        aggregation=lambda r, c: 0.85 * sum(r) - 0.0001 * c[-1]
    )

    # Add to system
    system = System(components=[env, agent, obj_fn])
    system.run(steps=1000)

    I think you can make the obejctive functional reconfigurable of course, but push the complexity mostly into the component.

    So we definetly do not actually want to build a static graph of operations like keras. I think the
    power comes from the flexibility, freedom, expressiveness you can achieve within each component. The only structure
    we need to impose is this component port based message passing. Phyilopshically this is just like ROS..


---
Philosophy
Graph at component level, complexity inside components.

Components are self-contained (complex Python ok internally)
System space: ONLY port wiring (clean!)
No building objectives from external specs
Like ROS: flexible components, port-based messaging

--


"""
