# System Visualization Workflow

## 🎯 The Complete Flow

### 1. **Right-Click on System Class** → Visualize

```python
# In your editor: lk/common/system.py
class MyRobotSystem:
    def __init__(self):
        self.robot = RobotDescription()  # ← Right-click here
        self.controller = PIDController()
        self.sensor = LaserSensor()
        self.env = SimulationWorld()
```

**User Action:** Right-click `MyRobotSystem` → **"🤖 Visualize System in Graph"**

---

### 2. **Python Backend Traces System**

```python
# system_tracer.py does:
1. Parse MyRobotSystem class using AST
2. Find all self.* = Component() assignments
3. Extract component names, types, line numbers
4. Build graph: System → Components
5. Return JSON to VS Code
```

**Output:**
```json
{
  "system_class": "MyRobotSystem",
  "components": [
    {"name": "robot", "type": "RobotDescription", "line": 45},
    {"name": "controller", "type": "PIDController", "line": 46},
    {"name": "sensor", "type": "LaserSensor", "line": 47},
    {"name": "env", "type": "SimulationWorld", "line": 48}
  ],
  "graph": {
    "nodes": [...],
    "edges": [...]
  }
}
```

---

### 3. **React Flow Renders Graph**

```
┌────────────────────────┐
│  🤖 MyRobotSystem      │  ← Input node (the system)
└───────┬────────────────┘
        │
    ┌───┴───┬───────┬──────┐
    │       │       │      │
    v       v       v      v
  ┌───┐   ┌───┐  ┌───┐  ┌───┐
  │🤖 │   │⚙️ │  │📡│  │🌍│  ← Component nodes
  └───┘   └───┘  └───┘  └───┘
  robot  ctrl   sensor  env
```

---

### 4. **Click Node** → Opens File at Exact Line

```typescript
// User clicks "robot" node
onNodeClick(node) {
    // node.data = {filePath: "system.py", line: 45}
    vscode.openTextDocument(filePath).then(doc => {
        vscode.showTextDocument(doc, {line: 45});
    });
}
```

**Result:** Cursor jumps to `self.robot = RobotDescription()` in your editor!

---

### 5. **Edit Code** → Graph Auto-Reloads

```python
# You edit system.py:
class MyRobotSystem:
    def __init__(self):
        self.robot = RobotDescription()
        self.controller = PIDController()
        self.sensor = LaserSensor()
        self.env = SimulationWorld()
        self.actuator = MotorController()  # ← ADD THIS
        # Save file (Ctrl+S)
```

**Auto-reload triggers:**
1. File watcher detects change
2. Python backend re-traces system
3. Graph updates automatically
4. New "actuator" node appears!

---

## VS Code Superpowers 🚀

### Feature 1: Go to Definition from Graph

```typescript
// Double-click component node
onNodeDoubleClick(node) {
    const componentType = node.data.componentType; // "RobotDescription"
    
    // Use VS Code's built-in symbol search
    vscode.executeCommand(
        'vscode.executeWorkspaceSymbolProvider',
        componentType
    ).then(symbols => {
        // Jumps to class RobotDescription definition!
    });
}
```

**Result:** Double-click "robot" → Opens `robot_description.py` at `class RobotDescription:`

---

### Feature 2: Hover for Details

```typescript
// Hover over component node
onNodeMouseEnter(node) {
    // Show tooltip with:
    // - Component type
    // - File location  
    // - Method list (from AST parsing)
    // - Dependencies
}
```

---

### Feature 3: Find All References

```typescript
// Right-click node → "Find References"
vscode.executeCommand(
    'vscode.executeReferenceProvider',
    uri,
    position
);
// Shows all places that use self.robot
```

---

### Feature 4: Live Code Awareness

```typescript
// Track what file user is viewing
vscode.window.onDidChangeActiveTextEditor(editor => {
    const currentFile = editor.document.uri.fsPath;
    
    // Highlight corresponding node in graph
    webview.postMessage({
        type: 'highlightNode',
        filePath: currentFile
    });
});
```

**Result:** Navigate to `sensor.py` → Sensor node highlights in graph!

---

## 🔄 Development Workflow

### Fast Iteration (Web Mode):
```bash
# Terminal 1: Python backend
cd python_backend
python system_tracer.py /path/to/workspace

# Terminal 2: React dev server
cd webview-ui
npm run dev

# Browser: http://localhost:5173
# Make changes → See immediately
```

### VS Code Integration (When Ready):
```bash
# Build once
cd webview-ui && npm run build
cd .. && npm run compile

# Press F5
# Now you get auto-reload + deep editor integration!
```

---

## Why This is Powerful 💡

### Traditional Approach:
1. Read code to understand system structure
2. Mentally build component tree
3. Jump between files manually
4. Lose context when editing

### With System Visualizer:
1. **Right-click** → See entire system instantly
2. **Click node** → Jump to exact line
3. **Edit code** → Graph updates automatically  
4. **Double-click** → Go to component definition
5. **Visual context** → Always know system structure

---

## Advanced: Trace Component Dependencies

```python
# system_tracer.py can also trace:
def trace_component_dependencies(component_type: str):
    """
    Find what this component depends on
    """
    # Parse component's __init__
    # Find its self.* = SubComponent()
    # Recursively build dependency tree
    # Return nested graph
```

**Result:** Right-click "Controller" → See what IT uses internally!

---

## Example: Real System

```python
class CartPoleSystem:
    def __init__(self):
        self.env = GymEnvironment("CartPole-v1")
        self.agent = DQNAgent(
            state_size=4,
            action_size=2
        )
        self.replay_buffer = ReplayBuffer(10000)
        self.plotter = LivePlotter()
```

**Visualization:**
```
     ┌──────────────────┐
     │ 🤖 CartPoleSystem │
     └────────┬──────────┘
              │
    ┌─────────┼──────────┬─────────┐
    │         │          │         │
┌───v───┐ ┌──v──┐  ┌────v─────┐ ┌─v──┐
│🌍 env │ │🎬agent│ │💾 buffer │ │📊plot│
└───────┘ └─────┘  └──────────┘ └────┘
```

**Click any node** → Jump to code!
**Edit system** → Graph updates!
**Double-click** → See component definition!

---

## Bottom Line

✅ **Right-click** → Instant visualization  
✅ **Auto-parse** → Python AST extraction  
✅ **Click nodes** → Jump to code  
✅ **Auto-reload** → Edit code, graph updates  
✅ **VS Code integration** → Go to definition, find references, etc.

**You get a living, breathing view of your system that stays in sync with your code!** 🚀


