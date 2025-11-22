# do we want open ai gym as a dependency? I think we can take the ideas but implement our own simpler verison...


"""
Ok I think its fine to have gym and a depency

At the env level we can have differen pip install[optoins], and gym is a classic on. Its like downloading robot_descriptions to get started

I like having the light weight datasets (synthetic essentially beacuse we can easily generate, and delete the dataafterwards etc.)



This is a good place to start beacuse the simplest possible system would have just two components.

I am finding perhaps just the hardest thing here now is to quickly set oup the constraints and objective function..

"""

# Good defaults?
agent = Agent()
env = Env()

# If we use factory methods than no type hinting... so lets just Agent Class Directly.
agent.config.type = "random_agent"
agent.config. = "gym_env"

action = agent(env.obs)
obs = env(action)

system = System(nodes=[agent, env], components=[agent, env], outputs=[action, obs])

sytem.summary()
system.set_objective()
system.launch()
