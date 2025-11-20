# LK Viewer Implementation Summary

## What I Built for You

A **minimal, functional viewer** that demonstrates reactive visualization with Python → Viewer updates.

### Files Created:

1. **`simple_rr_robot.py`** - Simple 2-DOF robot description
2. **`viewer_server.py`** - FastAPI server with WebSocket & file watching
3. **`viewer.html`** - Three.js-based 3D viewer
4. **`requirements.txt`** - Dependencies
5. **`QUICKSTART.md`** - How to use it

## Architecture Decision: Option 2 (Full Regeneration)

### Why Start Here?

✅ **Simple** - Easiest to implement and reason about  

✅ **Functional** - Solves the immediate problem  

✅ **Fast to build** - You can iterate quickly  

✅ **Foundation** - Easy to upgrade later if needed

### Trade-offs Accepted:

❌ Loses formatting/comments when editing from viewer (Phase 2)  

❌ Requires discipline to keep Python as source of truth

But for **Phase 1** (Python → Viewer), these don't matter!

## How It Works

```
┌─────────────────────┐
│  Python Source      │  You edit this and as soon
│  simple_rr_robot.py │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  File Watcher       │  Detects changes
│  (watchdog)         │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Dynamic Import     │  Reloads Python module
│  & Serialize        │  Calls .to_dict()
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  WebSocket Push     │  Sends JSON to browser
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Three.js Viewer    │  Re-renders 3D scene
│  (browser)          │
└─────────────────────┘
```

## Try It Now!

```bash
cd lk_viewer
pip install -r requirements.txt
python viewer_server.py
# Open http://localhost:8000
# Edit simple_rr_robot.py and watch it update!
```

## Next Steps for YOUR Robot (BamFbDev)

To visualize `bam_fb_dev.py`:

### 1. Add `to_dict()` method

The viewer needs a JSON representation. You'll need to add a method like:

```python
def to_dict(self) -> dict:
    return {
        "name": "BamFbDev",
        "type": "robot",
        "links": [...],  # Serialize your links
        "joints": [...], # Serialize your joints
    }
```

### 2. Point server to your file

In `viewer_server.py`, change:

```python
robot_file = Path("/path/to/bam_fb_dev.py")
```

### 3. Enhance viewer for meshes (optional)

Current viewer handles basic shapes (cylinder, box, sphere). For STL meshes, you'll need to add STL loading to `viewer.html`.

## For Phase 2: Viewer → Python (Later)

When you're ready for bidirectional editing:

### Easy Wins (Start Here):

- **Sliders** for numeric values (joint angles, link lengths)
- **Color pickers** for visual properties
- Send updates via WebSocket to server
- Server updates Python file using AST or templates

### Medium Difficulty:

- **Dropdowns** for enums/choices
- **Text inputs** for names/labels

### Hard (Only if Needed):

- **Adding/removing links** (requires code generation)
- **Reordering** (complex AST manipulation)

### Regeneration Strategy:

```python
# Add metadata to JSON
{
    "__source__": {
        "file": "bam_fb_dev.py",
        "class": "BamFbDev",
        "editable_fields": ["joint1_angle", "link1_length"]
    },
    # ... rest of robot data
}
```

When user edits:

1. Viewer sends update to server
2. Server parses Python file with AST
3. Server updates specific fields
4. Server writes back to file
5. File watcher triggers reload → back to Phase 1 flow!

## Philosophy Alignment

✅ **Make software for ourselves** - Tool to solve your problem  

✅ **Simplest, fastest way** - ~200 lines total, works in 5 min  

✅ **Purely functional** - No over-engineering, just what you need

## What This Gives You

1. **Instant visual feedback** when editing robot descriptions
2. **Foundation** for bidirectional editing (when you need it)
3. **Template** to extend to other systems (Gym envs, etc.)

Start with this, use it, see what you actually need, then evolve it!
