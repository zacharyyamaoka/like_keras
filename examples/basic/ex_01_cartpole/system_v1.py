

"""

Intial skeleton code we are writing, next iteration can go into system_v2, etc... We can keep them all here locally as memory, We can store the metrics next to them, becomes like a mini datbase..

Its helpful to view logs in weights and biases for humans, but for LLMs nice to have all the context in one place.

When you generate a new system, please leave the old one exactly as it is, this allows us to easily see progression!

Once we get alot of systems we can do Non maximum suppresion per say, and get rid of edits.. we can specific to keep only the version with the 5 highest performances...

# this is essentially liek saving the weights of the models so you can look them up later.

What is the simplest agent we could possibly make?

We have "templates", which are factories for different type of code we can completely reconfigure

Based on: https://gymnasium.farama.org/index.html

Basically this code should

- create cart pole agent
- create a cart pole env
- 1 describe this entire system in terms of components
- We should be able to bring up a graph in vs code to see the entire system, just like you press the button to view a urdf
(https://github.com/MorningFrog/urdf-visualizer). This is similar to a markdown renderer: https://github.com/jonathanyeung/mark-sharp
Its just a different "view" of the same underyling system.

- Christian Henkel has been particuarlly influencial on our views on diagonistics: https://roscon.ros.org/2024/talks/How_is_my_robot_-_On_the_state_of_ROS_Diagnostics.pdf
- https://github.com/ros/diagnostics

What does success look like? and how are we going to write a test to verify that it works?

How we know that the code works? If we achieve the target reward.
This is a specific care of diagonistics, which are quantiative metrics, we continually run to montior performance

"""

import gymnaisum as gym


agent_config = Agent.Config()
agent_config.type = "CartPole-v1"

env_config = Env.Config()
config.type = "CartPole-v1"

agent = Agent.from_config(agent_config)
env = Env.from_config(env_config)

#careful... can you do this? this is ok in python scripting but not really ok outside.. like how can we compile this?
# Ok lets leave it for now and see the implications..
while env.not_dont()

# You can use the components as normal python
action = agent(env.obs)
obs = env(action)
print("obs", obs)


# You can also build a system, and use that as normal python
system = System(nodes=[agent, env], components=[agent, env], outputs=[{"action": agent.action, "obs": env.obs}])

system.tick()
print("obs", system.out.get('obs')) # can address with .obs .get('obs') or ['obs'], beacuse this is dynamic I do prefer just the dynamic



# We can also "compile it to a static graph"
static_system = system.compile()

# Pruning is important to allow for faster LLM iterations when the "success" metric takes time 
# 1. Get rid of the component if you don't actually need it
# 2. Try to make feedback as fast as possible
# 2. Pruning/Parraleize/etc.
static_system.launch().enable_pruning("diangoistic.id", PrunerType).llm_monitor("diangoistic.reward", LLMMonitorType)

# Probably we need to change the order of the chaning as launch is a blocking call 
static_system.enable_pruning("diangoistic.id", PrunerType)
static_system.llm_monitor("diangoistic.reward", LLMMonitorType)
static_system.launch()

#namespace class within the system that handles all pruning concerns
# actuall is it better to directly call the top level system so that it doesn't matter how its implemented within?
# Hmm I guess it comes down to how large you want to make the interface
static_system.pruner

# Each system can have 1 high level diagnostic that we can monitor? or combine?
# This will monitor the system performance and shut the system down if its not performing well...
# Its cool how the entire system then becomes a self learning agent, which I think makes sense. There is an env and an agent in the env...
# the fun challenge now is not how long to train the cart pole agent, but how long to make a script that can train the cart pole agent...
# The better the inital prompt, its like providing a good seed for the optimization, like a starting point for the LLM.
# For example I can just copy paste the code from the website that shows the working examples...

# We can visualize static and dynamic systems easily a flow. 
# In the first view we see all the systems, sub systems, and components, sub components, and the connections between them.
# You can just have 1 connection line to show depency and direction, or enable all the connection lines for each dataflow.

# Even though this is a super simple example, you can refer to this in the future as a GymAgent system and configure it with GymAgent.Config. Right now
# This save just like 5 lines of code, but you can imagine, that you can have very complex systems.