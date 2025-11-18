# BAM
from .VisualObject import VisualObject
from .Frame import Frame
# PYTHON
from dataclasses import dataclass, field


@dataclass
class CompoundVisualObject(VisualObject):
    """Compound visual object that contains multiple child visual objects.
    
    This allows grouping related visual objects together under a single name
    without requiring viewers to parse a new datatype - they just iterate
    through the children list and render each child object normally.
    """
    children: list[VisualObject] = field(default_factory=list)

    def __post_init__(self):
        super().__post_init__()

    def add(self, child: VisualObject):
        self.children.append(child)

    def extend(self, children: list[VisualObject]):
        self.children.extend(children)

    def set_visibility(self, visible: bool, update_children: bool = True) -> None:
        self.visible = visible
        if update_children:
            for child in self.children:
                child.set_visibility(visible)
    
    def build_model(self):
        """Build the compound visual object model.
        
        Clears existing children and adds a parent frame for visibility control.
        Subclasses should call super().build_model() at the start of their own
        build_model() method, then add their specific children.
        """
        # Clear existing children
        self.children.clear()
        
        # Add parent frame for visibility control
        parent_frame = self.build_parent_frame()
        self.children.append(parent_frame)
    
    def build_parent_frame(self) -> VisualObject:
        """Create a parent frame for grouping children in the scene hierarchy.
        
        This frame acts as a visibility group - when drawn in viser, all children
        under this frame's namespace will be controlled by the frame's visibility.
        
        Returns:
            Frame object with identity transform at origin
        """
        
        return Frame(
            name=self.visual_id,
            show_axis=False,
            show_origin=False,
            visible=self.visible
        )


if __name__ == "__main__":
    compound = CompoundVisualObject(name="test_compound")
    print(compound)

