import lk
from dataclasses import dataclass

"""
You can specificy every single config here required to rebuild the entire system!
"""


# Named entity that propagates name to Nodes...
@dataclass
class NodeConfigs(lk.NodeConfigs):
    pass


@dataclass
class MyFavoriteSystemConfig(lk.System.Config):
    nodes: NodeConfigs = NodeConfigs()
