# BAM
from bam.utils import instance_to_snake, to_jsonable

# PYTHON
from dataclasses import dataclass, asdict
from collections import defaultdict
from itertools import count
import json


class VisualId(str):
    """Unique identifier string assigned to each visual object."""
    pass

@dataclass
class VisualObject:
    name: str = "" # Unique human readable name with slash notation (ex. /layer1/sublayer1/object)

    visible: bool = True

    _counters = defaultdict(lambda: count(1))

    @property
    def visual_id(self) -> VisualId:
        if self.name.startswith('/'):
            return VisualId(self.name)
        return VisualId(f"/{self.name}")

    def __post_init__(self):
        if self.name == "":
            self.name = self._next_id()

        # I think just simpler to read from the class name or isinstance?
        # if self.type == "":
        #     self.type = instance_to_snake(self)

    @classmethod
    def _next_id(cls) -> str:
        return f"{cls.__name__.lower()}_{next(cls._counters[cls])}"

    def set_visibility(self, visible: bool) -> None:
        self.visible = visible

    def __str__(self) -> str:
        return json.dumps(to_jsonable(asdict(self)), indent=2)

    def dump_to_yaml(self) -> str:
        # Save the YAML dump to a file in the same directory as this script.
        import yaml
        import os

        yaml_str = yaml.dump(asdict(self), sort_keys=False, allow_unicode=True)
        current_file_path = os.path.abspath(__file__)
        dir_path = os.path.dirname(current_file_path)
        base_name = os.path.splitext(os.path.basename(current_file_path))[0]
        object_name = self.name.replace('/', '_').strip('_') or "visual_object"
        file_name = f"{object_name}.yaml"
        file_path = os.path.join(dir_path, file_name)

        with open(file_path, "w", encoding="utf-8") as f:
            f.write(yaml_str)

        print(f"YAML file saved to: {file_path}")
        return yaml_str

if __name__ == "__main__":
    visual_object = VisualObject()
    print(visual_object)
    print(visual_object.visual_id)

    visual_object = VisualObject(name="test")
    print(visual_object.visual_id)

    visual_object = VisualObject(name="/test")
    print(visual_object.visual_id)

    visual_object = VisualObject(name="/layer1/sublayer1/object")
    print(visual_object.visual_id)