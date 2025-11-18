from dataclasses import dataclass, field

@dataclass
class MdpObservation:    
    terminated: bool = False
    truncated: bool = False

    def is_episode_over(self) -> bool:
        return self.terminated or self.truncated