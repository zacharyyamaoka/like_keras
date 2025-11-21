from dataclasses import dataclass


@dataclass
class ObjInfo:
    id: int = 0  # 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, etc.
    name: str = ""
    type: str = ""  # "syringe", "bottle", "blister_pack", "book", "etc."
