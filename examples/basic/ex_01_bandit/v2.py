# do we want open ai gym as a dependency? I think we can take the ideas but implement our own simpler verison...


"""
Ok I think its fine to have gym and a depency

At the env level we can have differen pip install[optoins], and gym is a classic on. Its like downloading robot_descriptions to get started

I like having the light weight datasets (synthetic essentially beacuse we can easily generate, and delete the dataafterwards etc.)



This is a good place to start beacuse the simplest possible system would have just two components.

I am finding perhaps just the hardest thing here now is to quickly set oup the constraints and objective function..

"""

# Good defaults?
# Random agent just needs to know what the action space is of the env. regardless of the obs it will always just sample a valid action
# agent = RandomAgent()
agent = RandomAgent.from_id("RandomAgent-v1")
# agent = Agent.from_config(Agent.Config(id="random_agent"))

# This can be implented /home/bam/bam_ws/src/like_keras/lk/agent/random_agent.py
env = Env.from_id("ClassBandit-v1")
# Many ways to construct an env, a classic on is the from_id, all envs we can register with a unique id, that allows you to look them up!

# If we use factory methods than no type hinting... so lets just Agent Class Directly.


# Is it ok having this interaction between code that is not through the inputs/outputs?
# I hope so, as this is the nice pythonic way we like.
# How can we recreate this though from a config file? I Think the trick is to capture the changes into the config file!
# When you save it it doesn't matter how the code became that way.
# So I think there are kinda two steps, one if the "setup" the other is running it. In setup phase we are free to basically do anything
# In pure pythonic we can can then just continue executing as all thoose variables will live in RAM.
# If we want to load up this system later, we can save the config, and now we actually don't need all that python construction code any more...
# In this sense what the python code is doing is helping us setup the system.
# When we actually run it it will just be thing


# the type of these spaces should our data distribution classes we were developing:
# /home/bam/bam_ws/src/like_keras/lk/msgs/random_msgs/data_dist.py
# These work basically indentically to gymasium spaces:
# https://gymnasium.farama.org/api/spaces/

agent.config.action_space = env.action_space

# Internally the env can call .reset if it has done that already.
# The agent doesn't know or control this at all, it just gets a stream of observation
for i in range(10):
    action = agent(env.obs)
    obs = env(action)
    print(obs.reward)  # we need a standard type of the observation...
    # how can I balance python usability with doing something like ClientResponse
    # /home/bam/bam_ws/src/like_keras/lk/msgs/lk/api/client_response.py
    # Client response is great if we are expeting failures, but it does add overhead if we are not...


# Options. switch between returning client mode and not? Then are are two potetially return types
# Advantage is you can directly acess disadvanatge is that iuts no longer standard.
# One big architecrue decision would be to enforce that each component value is accessed eseential via a Client, or we can call it PortResponse..
# Th ebenfit of this is that you really can abstract away where the data is coming from, and we see the v

# system = System(nodes=[agent, env], components=[agent, env], outputs=[action, obs])

# sytem.summary()
# system.set_objective()
# system.launch()
