import lk
from dataclasses import dataclass

from lk.node import Node
"""
You can specificy every single config here required to rebuild the entire system!

We will us name spacing when we import...

Our goal here is to essentiall mimic a yaml static configure but for every single param in the system!

Even dignaotics...
"""

# Named entity that propagates name to Nodes...

# This is a way to explicity define the system config in Python
# @dataclass
# class NodeConfigs(lk.NodeConfigs):
#     node: Node.Config = field(default_factory=lambda: Node.Config())

# @dataclass
# class SystemConfig(lk.SystemConfig):
#     nodes: NodeConfigs = NodeConfigs()


If you want to define in a completely static way. you can do it just with the configs...


node1 = Node.Config(name="node1")


System.Config(nodes=[sensor_node, world_model_node, actor_node, actuators_node], components=[sensors, world_model, actor, actuators])

then you can compile it.

Each Config is associate with a class... I think we can seamless flip between the config and the class.

Actually its more pythonic though in my opinion to define the class it self. and then turn into a config..

That gives you type hinting not only on the config but also on all the class functions.. as the config can be apart of the class, but the class is not
apart of the config. I had this issuse with diagonistics...

System.to_config()

