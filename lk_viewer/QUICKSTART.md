# LK Viewer - Quick Start

## Philosophy: Start Simple, One Direction

**Phase 1 (Current):** Python → Viewer (Reactive Viewing)
- Edit Python code → Auto-reload → Viewer updates
- Simple, functional, gets the job done

**Phase 2 (Later):** Viewer → Python (Bidirectional Editing)

## Quick Start

### 1. Install dependencies:

```bash
cd lk_viewer
pip install -r requirements.txt
```

### 2. Run the server:

```bash
python viewer_server.py
```

### 3. Open your browser:

```
http://localhost:8000
```

### 4. Edit the robot:

Open `simple_rr_robot.py` and change values:

```python
robot = SimpleRRRobot(
    link1_length=0.8,  # Change this
    link2_length=0.5,  # Change this
    joint1_angle=0.5,  # Change this
)
```

Save the file and watch the viewer update automatically! 🎉

## How It Works

```
simple_rr_robot.py (Python Source)
    ↓
    File watcher detects changes
    ↓
    viewer_server.py reloads and serializes to JSON
    ↓
    WebSocket pushes update to browser
    ↓
    viewer.html (Three.js) re-renders
```

## Next Steps for Your BamFbDev Robot

To visualize your `bam_fb_dev.py`:

1. Add a `to_dict()` method to serialize it to JSON
2. Point `viewer_server.py` to your robot file
3. Edit joint angles, link lengths, etc. and see live updates

## Trade-offs Discussion

### Option 1: Smart Preservation
- ✅ Preserves formatting, comments
- ❌ Complex to implement
- ❌ Hard to handle new fields

### Option 2: Full Regeneration (Current)
- ✅ Simple, predictable
- ✅ Easy to implement
- ❌ Loses formatting, comments
- **Decision:** Start here for v1

### Option 3: Hybrid
- ✅ Best of both worlds
- ❌ Most complex
- **Decision:** Only if needed later

## Principles

1. **Make software for ourselves** - This is a tool, not a product
2. **Simplest, fastest way** - No over-engineering
3. **Purely functional** - Solve the problem at hand

## For Later: Bidirectional Editing

When you're ready to add Viewer → Python:

1. **Easy wins first:** Numeric sliders (joint angles, link lengths)
2. **Medium:** Color pickers, dropdowns
3. **Hard:** Adding/removing links (code generation)

For regeneration approach:
- Store metadata in JSON: `"__source__": {"file": "bam_fb_dev.py", "class": "BamFbDev"}`
- Viewer edits → Update JSON → Regenerate Python class
- Use AST manipulation or templates for code generation

