# Bidirectional Graph Editor - Like Mark Sharp!

## How Mark Sharp Works (and how we can do it for graphs):

[Mark Sharp](https://marketplace.visualstudio.com/items?itemName=jonathan-yeung.mark-sharp) uses VS Code's **Custom Editor API** to create a WYSIWYG markdown editor with **bidirectional editing**:

### Key Components:

1. **Custom Editor API** (`vscode.CustomTextEditorProvider`)
   - Registers as the editor for `.md` files
   - Intercepts file opening
   - Provides custom UI (webview)

2. **TextDocument Synchronization**
   - Watches for text changes → Updates visual
   - Visual changes → Applies edits to text
   - Uses `vscode.WorkspaceEdit` for atomic updates

3. **Two-Way Binding**
   ```
   Edit Visual ──→ Generate Code ──→ Apply Edit to File
                      ↑                      ↓
                      └──────────────────────┘
                            File watches for changes
   ```

## Your LK Graph Editor Implementation:

### Architecture:

```
┌─────────────────────────────────────────────────────┐
│                  VS Code Window                     │
├─────────────────────────────────────────────────────┤
│                                                     │
│  Python Code (system_config.py)                    │
│  ┌─────────────────────────────────────────────┐   │
│  │ robot = Component(                          │   │
│  │     name="robot",                           │   │
│  │     position=(100, 200)                     │   │
│  │ )                                           │   │
│  │                                             │   │
│  │ robot.connect(sensor)                       │   │
│  └─────────────────────────────────────────────┘   │
│              ↕ Custom Editor API ↕                  │
│  ┌─────────────────────────────────────────────┐   │
│  │  React Graph Visualization                  │   │
│  │  ┌───────┐     ┌────────┐                   │   │
│  │  │ Robot │────→│ Sensor │                   │   │
│  │  └───────┘     └────────┘                   │   │
│  │  (Drag to move, click to edit)              │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
```

### Flow:

**User drags node:**
1. React Flow detects drag
2. Sends message to extension: `{ type: 'nodeMoved', nodeId: 'robot', position: {x: 150, y: 250} }`
3. Extension finds the line: `robot = Component(...)`
4. Creates `WorkspaceEdit` to update `position=(150, 250)`
5. VS Code applies edit
6. File system notifies change
7. Extension re-parses Python → Sends updated graph data to webview

**User edits Python code:**
1. User types: `position=(200, 300)`
2. `onDidChangeTextDocument` fires
3. Extension parses Python code
4. Extracts graph structure (nodes, edges, positions)
5. Sends `updateGraph` message to webview
6. React Flow updates visualization

### Implementation Steps:

1. **Create Custom Editor Provider**
   - Register for `*_config.py` files
   - Provide webview with graph visualization

2. **Python AST Parsing**
   - Parse Python code to extract:
     - Component definitions
     - Connections/ports
     - Positions/metadata
   - Convert to graph data structure

3. **Code Generation**
   - Graph changes → Generate Python code
   - Use `WorkspaceEdit` to apply changes
   - Maintain code structure/comments

4. **React Graph UI**
   - Use React Flow for visualization
   - Send all edits back to extension
   - Receive updates from file changes

### Key Advantages:

✅ **Single Source of Truth** - Python file is the source
✅ **Version Control** - All changes are file changes (git diff works!)
✅ **No Sync Issues** - VS Code handles file consistency
✅ **Instant Updates** - Both views stay in sync automatically
✅ **Familiar Workflow** - Use regular text editor when needed

### Example Usage:

```python
# system_config.py

from lk import Component, System

# Create components (positions managed by visual editor)
robot = Component(
    name="robot",
    position=(100, 200),  # Updated by dragging in graph view
    config={
        "speed": 1.0
    }
)

sensor = Component(
    name="sensor", 
    position=(300, 200)
)

# Connect components (created by drawing edges in graph view)
robot.connect(sensor)

system = System([robot, sensor])
```

**Right-click file → "Open With..." → "LK Graph Editor"**

Now you see a visual graph! Drag nodes, draw connections, all updates the Python code!

### Next Steps:

1. **Implement Python Parser**
   - Use Python's `ast` module
   - Extract component structure
   - Handle complex Python patterns

2. **Build React Graph UI**
   - Embed React Flow in webview
   - Handle user interactions
   - Send updates to extension

3. **Code Generator**
   - Convert graph changes to Python code
   - Preserve formatting/comments
   - Handle complex edits

### Reference:

- [VS Code Custom Editor API](https://code.visualstudio.com/api/extension-guides/custom-editors)
- [Mark Sharp Source](https://github.com/jonathanyeung/mark-sharp)
- Your existing `viewer_server.py` can be adapted into the webview!

This is **exactly** how tools like Mark Sharp, Jupyter notebooks in VS Code, and other visual editors work!


