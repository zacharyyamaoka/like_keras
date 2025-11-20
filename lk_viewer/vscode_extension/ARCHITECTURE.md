# LK Viewer Architecture: Integrated vs Standalone

## The Key Question: How Does Backend Access Code?

### Option 1: Filesystem Access (Current - Recommended) ✅

```python
# Python backend reads files directly
WORKSPACE_ROOT = "/home/bam/bam_ws/src/like_keras"
for py_file in Path(WORKSPACE_ROOT).rglob("*.py"):
    with open(py_file) as f:
        code = f.read()
        # Parse with ast, analyze, build graph
```

**How it works:**
1. VS Code extension spawns Python subprocess
2. Passes workspace path as argument: `python server.py /path/to/workspace`
3. Python reads files directly from disk
4. No network transfer of code needed

**Advantages:**
- ⚡ **Fast**: Direct file I/O, no network latency
- 🔒 **Secure**: Code never leaves your machine
- 📁 **File watching**: Can use `watchdog` to detect changes automatically
- 💾 **Memory efficient**: Stream large files, no need to load all in RAM
- ✏️ **Direct edits**: Python can write back to files if needed

**When to use:**
- Local development (99% of use cases)
- Want instant updates when files change
- Working with large codebases
- Need to modify files from graph UI

---

### Option 2: API-Based (Standalone Mode) 🌐

```python
# Python backend receives code via POST request
@app.post("/parse")
async def parse(submission: CodeSubmission):
    # submission.files = [{"path": "x.py", "content": "code..."}]
    for file_data in submission.files:
        code = file_data['content']
        # Parse, analyze, build graph
```

**How it works:**
1. Run Python server independently: `python server.py` (no args)
2. VS Code extension or web app sends code via HTTP POST
3. Python stores code in memory or temp files
4. Returns graph data

**Advantages:**
- 🌍 **Remote**: Backend can run on different machine/server
- 🚀 **Deployed**: Can be a web service accessible to team
- 🔗 **Language agnostic**: Any client can send code (not just VS Code)
- 📊 **Centralized**: One backend, many clients

**When to use:**
- Want web-based UI accessible from browser
- Backend on powerful server, VS Code on laptop
- Team collaboration (multiple users)
- Integration with CI/CD pipelines

---

## Hybrid Approach: Best of Both Worlds 🎯

The `server_hybrid.py` supports **both modes**:

```bash
# Integrated mode (with VS Code)
python server.py /path/to/workspace

# Standalone mode (web server)
python server.py  # No args = standalone
```

### Usage Examples:

#### Integrated (Current Setup):
```typescript
// VS Code extension spawns:
pythonProcess = spawn('python3', [
    'server.py', 
    workspaceRoot  // ← Filesystem path
]);

// React app fetches:
fetch('http://localhost:8765/parse')  // GET - reads from disk
```

#### Standalone (For web deployment):
```python
# Run server independently
python server.py

# Client sends code:
import requests
requests.post('http://localhost:8765/parse', json={
    'files': [
        {'path': 'system.py', 'content': 'class System: ...'},
        {'path': 'agent.py', 'content': 'class Agent: ...'}
    ]
})
```

---

## Hot Swapping Functionality 🔥

Since Python backend is **separate from VS Code extension**, you can:

```bash
# 1. Make changes to server.py
vim python_backend/server.py

# 2. Kill old process
pkill -f server.py

# 3. Restart (extension will auto-restart on next request)
# OR restart manually:
python python_backend/server.py /path/to/workspace
```

**No need to rebuild TypeScript/React!**

---

## Direct File Edits from Graph UI 💡

With filesystem access, you can add endpoints like:

```python
@app.post("/edit")
async def edit_file(path: str, line: int, new_content: str):
    """Edit file directly from graph UI"""
    with open(path, 'r') as f:
        lines = f.readlines()
    
    lines[line] = new_content + '\n'
    
    with open(path, 'w') as f:
        f.writelines(lines)
    
    return {"status": "updated", "path": path}
```

Then from React:
```typescript
// User edits node label in graph
const onNodeEdit = (nodeId, newLabel) => {
    fetch('http://localhost:8765/edit', {
        method: 'POST',
        body: JSON.stringify({
            path: node.data.filePath,
            line: node.data.line,
            new_content: newLabel
        })
    });
    
    // VS Code will auto-reload file!
};
```

---

## Recommendation for Your Use Case 💭

Based on "thin viewer, Python backend does parsing":

✅ **Use Integrated Mode (Filesystem)**
- Keep Python backend reading files directly
- Faster, simpler, more reliable
- Can still hot-swap Python code
- Easy to add file editing later
- Works perfectly for local development

🌐 **Add Standalone Mode Later** (if needed)
- When you want web-based UI
- When deploying for team use
- The hybrid server supports both!

---

## Current Status

Your setup is **already using filesystem access** via:
- Extension passes workspace path to Python
- Python reads `lk/` directory
- Builds graph from real files
- React renders it

**Just press F5 to test!** 🚀


