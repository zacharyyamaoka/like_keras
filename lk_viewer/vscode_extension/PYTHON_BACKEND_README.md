# LK Viewer with Python Backend

## Architecture

This VS Code extension uses a **3-tier architecture**:

```
┌─────────────────────────────────────────────────┐
│           VS Code Extension (TypeScript)         │
│  - Manages lifecycle                            │
│  - Spawns Python subprocess                     │
│  - Handles file opening commands                │
└────────────┬────────────────────────────────────┘
             │
             ├─ spawns ──────────────────────┐
             │                               │
             v                               v
┌────────────────────────────┐   ┌──────────────────────────────┐
│   Python Backend (FastAPI)  │   │  React Webview (Frontend)   │
│  - Parses Python codebase   │◄──│  - Renders React Flow graph  │
│  - Extracts AST structure   │   │  - Fetches from Python API   │
│  - Builds graph data        │   │  - Sends file open commands  │
│  - Serves on localhost:8765 │   │  - Interactive UI            │
└────────────────────────────┘   └──────────────────────────────┘
```

## Benefits

1. **Separation of Concerns**:
   - Python does heavy parsing/analysis
   - TypeScript manages VS Code integration
   - React provides rich UI

2. **Performance**:
   - Python backend runs in separate process
   - Can do expensive parsing without blocking UI
   - Can cache results

3. **Flexibility**:
   - Easy to add more analysis features in Python
   - Can reuse existing Python parsing code
   - Backend can be developed/tested independently

4. **Similar to Language Servers**:
   - Same pattern as Python Language Server, TypeScript Server, etc.
   - Industry-standard approach

## Files

- `src/extension.ts` - Main extension (spawns Python backend)
- `python_backend/server.py` - FastAPI server for parsing
- `webview-ui/src/App.tsx` - React frontend (fetches from backend)

## Usage

### To use the current static version:
```bash
cd webview-ui
npm run build
cd ..
npm run compile
# Press F5 to debug
```

### To use with Python backend:

1. **Install Python dependencies**:
```bash
cd python_backend
pip install fastapi uvicorn
```

2. **Update extension.ts**:
```bash
# Rename extension_with_python.ts to extension.ts
mv src/extension_with_python.ts src/extension.ts
```

3. **Update App.tsx**:
```bash
cd webview-ui/src
# Rename App_with_backend.tsx to App.tsx
mv App_with_backend.tsx App.tsx
```

4. **Build and run**:
```bash
cd webview-ui
npm run build
cd ..
npm run compile
# Press F5 - Python backend will auto-start!
```

## API Endpoints

- `GET /` - Health check
- `GET /parse?directory=lk` - Parse directory and return graph
- `GET /file/{file_path}` - Get details about specific file

## Future Enhancements

- Add caching layer for parsed files
- Watch for file changes and update graph
- Add more sophisticated import analysis
- Support for inter-module dependencies
- Custom graph layouts based on code structure
- Integration with your existing BAM analysis tools


