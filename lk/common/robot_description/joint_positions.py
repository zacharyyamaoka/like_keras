"""

    Intelligent container for named positions
    - Allows your to compose with other descriptions

"""

from dataclasses import dataclass, field
from typing import Literal

@dataclass
class JointPositions:
    """
        Container for named robot configurations with lazy composition.
        
        Supports composable position management where composite robots iterate through
        component positions on-demand rather than merging data eagerly.
        
        Example:
            arm_positions = JointPositions(
                joint_names=["joint_1", "joint_2"],
                positions={"ready": [0.0, -1.57]},
                default_positions="initial"
            )
            
            composite = JointPositions.combine(arm_positions, hand_positions)
            ready_pos = composite.get("ready")  # Lazy evaluation
    """
    
    joint_names: list[str] = field(default_factory=list)
    unprefixed_joint_names: list[str] = field(default_factory=list)
    positions: dict[str, list[float]] = field(default_factory=dict)
    default_positions: str = "initial"
    composites: list['JointPositions'] = field(default_factory=list)
    prefixes: list[str] = field(default_factory=list)
    
    @property
    def is_composite(self) -> bool:
        """Check if this is a composite (has composites) vs a leaf node."""
        return len(self.composites) > 0

    @property
    def name_to_joint_index(self) -> dict[str, int]:
        return self.get_name_to_joint_index()
    
    #TODO deal with prefix....
    def get_name_to_joint_index(self) -> dict[str, int]:
        return {name: index for index, name in enumerate(self.joint_names)}

    def __post_init__(self):

        if len(self.unprefixed_joint_names) == 0:
            self.unprefixed_joint_names = self.joint_names

        if not self.is_composite:
            for name, pos in self.positions.items():
                if len(pos) != len(self.joint_names):
                    raise ValueError(
                        f"Position '{name}' has {len(pos)} values but "
                        f"expected {len(self.joint_names)} joints"
                    )
    
    def get(
        self, 
        name: str, 
        fill_missing: Literal["default", "none", "error"] = "default"
    ) -> list[float | None] | None:
        """
            Get named position with lazy composition.
            
            DescriptionArgs:
                name: Position name (e.g., "ready", "open", "close")
                fill_missing: What to do if position not found in a composite:
                    - "default": Use composite's default_positions (e.g., "initial")
                                If a component has no position, fills with None values
                    - "none": Return None if ANY composite missing
                    - "error": Raise KeyError if ANY composite missing
            
            Returns:
                Composed list of joint positions (may contain None) or None
        """
        if self.is_composite:
            return self._get_composite(name, fill_missing)
        else:
            return self._get_leaf(name, fill_missing)
    
    def _get_composite(
        self, 
        name: str, 
        fill_missing: Literal["default", "none", "error"]
    ) -> list[float | None]:
        """Get position from composite by iterating through composites."""
        result = []
        any_found = False
        for composite in self.composites:
            values = composite.get(name, fill_missing=fill_missing)
            if values is not None:
                result.extend(values)
                any_found = True
        return result if any_found else None
    
    def _get_leaf(
        self, 
        name: str, 
        fill_missing: Literal["default", "none", "error"]
    ) -> list[float | None]:
        """Get position from leaf node."""
        if name in self.positions:
            return self.positions[name]
        
        if fill_missing == "default":
            default_name = self.default_positions
            if default_name in self.positions:
                return self.positions[default_name]
            return None

        if fill_missing == "none":
            return None

        if fill_missing == "error":
            raise KeyError(f"Named position '{name}' not found")
        
        return None
    
    def has(self, name: str) -> bool:
        """Check if position exists (in any composite)."""
        if self.is_composite:
            return any(comp.has(name) for comp in self.composites)
        return name in self.positions
    
    def keys(self) -> set[str]:
        """Get all available position names (union of all composites)."""
        if self.is_composite:
            all_keys = set()
            for comp in self.composites:
                all_keys.update(comp.keys())
            return all_keys
        return set(self.positions.keys())
    
    def register(self, name: str, positions: list[float]):
        """Register a new named position (only for leaf nodes)."""
        if self.is_composite:
            raise RuntimeError("Cannot register positions on composite JointPositions")
        if len(positions) != len(self.joint_names):
            raise ValueError(
                f"Position has {len(positions)} values but "
                f"expected {len(self.joint_names)} joints"
            )
        self.positions[name] = positions

    def prefix(self, prefix: str):
        """Prefix all position names."""
        self.prefixes.append(prefix)
        prefixed_joint_names = ["".join(self.prefixes) + name for name in self.unprefixed_joint_names]
        self.joint_names = prefixed_joint_names


    def get_joint_names(self, prefix=True) -> list[str]:
        if prefix:
            return self.joint_names
        return self.unprefixed_joint_names
    
    @staticmethod
    def combine(composites: list['JointPositions']) -> 'JointPositions':
        """
            Create composite JointPositions from composites (no copying!).
            
            Positions are evaluated lazily on .get() calls, avoiding data duplication.
            
            Can be called with:
                combine([comp1, comp2, comp3])
        """
        all_joint_names = []
        all_unprefixed_joint_names = []

        # This works even with duplicate names!
        for comp in composites:
            all_joint_names.extend(comp.joint_names)
            all_unprefixed_joint_names.extend(comp.unprefixed_joint_names)
        
        return JointPositions(
            joint_names=all_joint_names,
            unprefixed_joint_names=all_unprefixed_joint_names,
            positions={},
            default_positions="initial",
            composites=list(composites)
        )

    def extend(self, *other_joint_positions: 'JointPositions') -> 'JointPositions':
        return JointPositions.combine(self, *other_joint_positions)