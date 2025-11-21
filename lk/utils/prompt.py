#!/usr/bin/env python3

"""
Prompting utilities for AI-compile workflows.

Helps generate prompts for cross-language compilation:
- Collects component/system specifications
- Gathers relevant API documentation
- Includes example datasets for verification
"""

# BAM
from lk.msgs.msg import Msg

# PYTHON
from dataclasses import dataclass, field
from typing import Optional, Any
from pathlib import Path


@dataclass
class Prompt:
    """
    Structured prompt for AI-assisted compilation.

    Contains all information needed to reimplement a component
    or function in a target language.
    """

    # Core specifications
    target_language: str  # 'cpp', 'rust', 'julia', etc.
    input_spec: dict[str, type]  # {'pose': PoseStamped, 'image': Image}
    output_spec: dict[str, type]  # {'action': Action, 'confidence': float}

    # Implementation details
    description: str = ""  # High-level description of what this does
    source_code: Optional[str] = None  # Original Python implementation
    component_name: Optional[str] = None  # Name of component if applicable

    # Context and documentation
    api_docs: dict[str, str] = field(
        default_factory=dict
    )  # {'eigen': 'docs...', 'ros2': 'docs...'}
    examples: list[str] = field(default_factory=list)  # Example code snippets

    # Verification data
    dataset_path: Optional[Path] = None  # Path to input/output dataset
    dataset_format: str = "pickle"  # 'pickle', 'mcap', 'json', etc.

    # Build/integration instructions
    build_instructions: Optional[str] = None  # CMakeLists, Cargo.toml, etc.
    dependencies: list[str] = field(default_factory=list)  # Required libraries

    # Metadata
    notes: str = ""  # Additional context or constraints

    def to_markdown(self) -> str:
        """
        Generate markdown-formatted prompt for AI code generation.

        Returns:
            Formatted prompt string ready to paste into AI assistant
        """
        sections = []

        # Header
        sections.append(f"# Compilation Request: {self.component_name or 'Function'}")
        sections.append(f"\nTarget Language: **{self.target_language}**\n")

        # Description
        if self.description:
            sections.append("## Description")
            sections.append(self.description)
            sections.append("")

        # Input/Output specification
        sections.append("## Interface Specification")
        sections.append("\n### Inputs")
        for name, type_hint in self.input_spec.items():
            type_name = (
                type_hint.__name__ if hasattr(type_hint, "__name__") else str(type_hint)
            )
            sections.append(f"- `{name}`: {type_name}")

        sections.append("\n### Outputs")
        for name, type_hint in self.output_spec.items():
            type_name = (
                type_hint.__name__ if hasattr(type_hint, "__name__") else str(type_hint)
            )
            sections.append(f"- `{name}`: {type_name}")
        sections.append("")

        # Source code
        if self.source_code:
            sections.append("## Original Python Implementation")
            sections.append("```python")
            sections.append(self.source_code)
            sections.append("```")
            sections.append("")

        # API docs
        if self.api_docs:
            sections.append("## Relevant API Documentation")
            for api_name, docs in self.api_docs.items():
                sections.append(f"\n### {api_name}")
                sections.append(docs)
            sections.append("")

        # Examples
        if self.examples:
            sections.append("## Example Code")
            for i, example in enumerate(self.examples, 1):
                sections.append(f"\n### Example {i}")
                sections.append("```")
                sections.append(example)
                sections.append("```")
            sections.append("")

        # Verification dataset
        if self.dataset_path:
            sections.append("## Verification Dataset")
            sections.append(f"- Path: `{self.dataset_path}`")
            sections.append(f"- Format: {self.dataset_format}")
            sections.append(
                "\nUse this dataset to verify your implementation matches the original."
            )
            sections.append("")

        # Dependencies
        if self.dependencies:
            sections.append("## Dependencies")
            for dep in self.dependencies:
                sections.append(f"- {dep}")
            sections.append("")

        # Build instructions
        if self.build_instructions:
            sections.append("## Build Instructions")
            sections.append(self.build_instructions)
            sections.append("")

        # Notes
        if self.notes:
            sections.append("## Additional Notes")
            sections.append(self.notes)
            sections.append("")

        return "\n".join(sections)

    def save(self, path: Path) -> None:
        """
        Save prompt to markdown file.

        Args:
            path: Output file path (will be .md)
        """
        path = Path(path)
        if path.suffix != ".md":
            path = path.with_suffix(".md")

        path.write_text(self.to_markdown())

    @classmethod
    def from_component(
        cls, component: Any, target_language: str, dataset_path: Optional[Path] = None
    ) -> "Prompt":
        """
        Create prompt from a component instance.

        Args:
            component: Component to generate prompt for
            target_language: Target language ('cpp', 'rust', etc.)
            dataset_path: Optional path to verification dataset

        Returns:
            Prompt instance ready for code generation
        """
        # Extract input/output specs from component ports
        input_spec = {}
        output_spec = {}

        if hasattr(component, "inputs"):
            for port_name, port in component.inputs._ports.items():
                input_spec[port_name] = port.msg_type

        if hasattr(component, "outputs"):
            for port_name, port in component.outputs._ports.items():
                output_spec[port_name] = port.msg_type

        # Get source code if available
        import inspect

        try:
            source_code = inspect.getsource(component.__class__)
        except (TypeError, OSError):
            source_code = None

        return cls(
            target_language=target_language,
            input_spec=input_spec,
            output_spec=output_spec,
            component_name=component.__class__.__name__,
            description=component.__class__.__doc__ or "",
            source_code=source_code,
            dataset_path=dataset_path,
        )


def create_prompt(
    component: Optional[Any] = None,
    target_language: str = "cpp",
    input_spec: Optional[dict[str, type]] = None,
    output_spec: Optional[dict[str, type]] = None,
    description: str = "",
    dataset_path: Optional[Path] = None,
    **kwargs,
) -> Prompt:
    """
    Create a compilation prompt.

    Can be used with a component or with manual specifications.

    Args:
        component: Optional component to extract specs from
        target_language: Target programming language
        input_spec: Manual input specification (if no component)
        output_spec: Manual output specification (if no component)
        description: What this code does
        dataset_path: Path to verification dataset
        **kwargs: Additional fields for Prompt dataclass

    Returns:
        Prompt instance

    Example with component:
        >>> prompt = create_prompt(
        ...     component=my_controller,
        ...     target_language='cpp',
        ...     dataset_path=Path('verification_data.pkl')
        ... )
        >>> prompt.save('compile_controller.md')

    Example with manual specs:
        >>> prompt = create_prompt(
        ...     target_language='rust',
        ...     input_spec={'x': float, 'y': float},
        ...     output_spec={'distance': float},
        ...     description='Euclidean distance calculation'
        ... )
    """
    if component is not None:
        return Prompt.from_component(
            component=component,
            target_language=target_language,
            dataset_path=dataset_path,
        )

    if input_spec is None or output_spec is None:
        raise ValueError("Must provide either component or input_spec/output_spec")

    return Prompt(
        target_language=target_language,
        input_spec=input_spec,
        output_spec=output_spec,
        description=description,
        dataset_path=dataset_path,
        **kwargs,
    )


def send_prompt(prompt: Prompt, method: str = "clipboard") -> None:
    """
    Send prompt to target destination.

    Args:
        prompt: Prompt to send
        method: Delivery method - 'clipboard', 'file', or 'print'

    Currently supports:
    - 'clipboard': Copy to system clipboard (requires pyperclip)
    - 'file': Save to file (uses component name)
    - 'print': Print to stdout

    Future: Could integrate with AI APIs directly
    """
    markdown = prompt.to_markdown()

    if method == "clipboard":
        try:
            import pyperclip

            pyperclip.copy(markdown)
            print(f"✓ Prompt copied to clipboard ({len(markdown)} chars)")
        except ImportError:
            print("Warning: pyperclip not installed. Printing instead.")
            print(markdown)

    elif method == "file":
        filename = f"compile_{prompt.component_name or 'function'}.md"
        Path(filename).write_text(markdown)
        print(f"✓ Prompt saved to {filename}")

    elif method == "print":
        print(markdown)

    else:
        raise ValueError(f"Unknown method: {method}")


if __name__ == "__main__":

    print("\n" + "=" * 70)
    print("Prompt Utilities for AI-Compile")
    print("=" * 70)

    # Example 1: Manual specification
    print("\nExample 1: Creating prompt from manual specifications")
    prompt = create_prompt(
        target_language="cpp",
        input_spec={"position": list, "velocity": list},
        output_spec={"acceleration": list},
        description="Simple PID controller for robot motion",
        notes="Should compile with C++17, use Eigen for linear algebra",
    )

    print("\nGenerated prompt preview:")
    print(prompt.to_markdown()[:400] + "...\n")

    # Example 2: With verification dataset
    print("\nExample 2: Prompt with verification dataset")
    prompt_with_data = create_prompt(
        target_language="rust",
        input_spec={"image": Any, "threshold": float},
        output_spec={"keypoints": list},
        description="Feature detection algorithm",
        dataset_path=Path("test_data/features_verification.pkl"),
        dependencies=["opencv-rust", "nalgebra"],
    )

    print(f"Dataset path: {prompt_with_data.dataset_path}")
    print(f"Dependencies: {prompt_with_data.dependencies}")

    print("\n" + "=" * 70)
    print("Use create_prompt() to generate compilation prompts")
    print("Use send_prompt() to deliver them (clipboard/file/print)")
    print("=" * 70 + "\n")
