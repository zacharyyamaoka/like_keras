# do we want open ai gym as a dependency? I think we can take the ideas but implement our own simpler verison...


"""
Ok I think its fine to have gym and a depency

At the env level we can have differen pip install[optoins], and gym is a classic on. Its like downloading robot_descriptions to get started

I like having the light weight datasets (synthetic essentially beacuse we can easily generate, and delete the dataafterwards etc.)



This is a good place to start beacuse the simplest possible system would have just two components.

"""

agent = Agent()
env = Env()

action = agent(env.obs)
obs = env(action)

system = System(nodes=[agent, env], components=[agent, env], outputs=[action, obs])

sytem.summary()
system.set_objective()
system.launch()
