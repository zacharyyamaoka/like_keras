"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
exports.activate = activate;
exports.deactivate = deactivate;
const vscode = __importStar(require("vscode"));
function activate(context) {
    console.log('🚀 LK Viewer extension is now active!');
    // Show a message to confirm activation
    vscode.window.showInformationMessage('LK Viewer Extension Activated! 🎉');
    // Register the webview provider for sidebar
    const provider = new LKViewerWebviewProvider(context.extensionUri);
    context.subscriptions.push(vscode.window.registerWebviewViewProvider('lk-viewer.panel', provider));
    // Command to open panel in editor area
    context.subscriptions.push(vscode.commands.registerCommand('lk-viewer.openPanel', () => {
        const panel = vscode.window.createWebviewPanel('lkViewer', 'LK Viewer - File Navigator', vscode.ViewColumn.Two, {
            enableScripts: true,
            retainContextWhenHidden: true
        });
        panel.webview.html = getWebviewContent(panel.webview, context.extensionUri);
        setupMessageHandling(panel.webview);
        vscode.window.showInformationMessage('LK Viewer panel opened!');
    }));
    // Test command to open a file directly
    context.subscriptions.push(vscode.commands.registerCommand('lk-viewer.openFile', async () => {
        const testPath = '/home/bam/bam_ws/src/like_keras/lk_viewer/features.md';
        await openFileAtLine(testPath, 10);
        vscode.window.showInformationMessage(`Opened ${testPath}:10`);
    }));
    console.log('✅ All LK Viewer commands registered');
}
class LKViewerWebviewProvider {
    constructor(_extensionUri) {
        this._extensionUri = _extensionUri;
    }
    resolveWebviewView(webviewView, context, _token) {
        webviewView.webview.options = {
            enableScripts: true,
            localResourceRoots: [this._extensionUri]
        };
        webviewView.webview.html = getWebviewContent(webviewView.webview, this._extensionUri);
        setupMessageHandling(webviewView.webview);
    }
}
function setupMessageHandling(webview) {
    webview.onDidReceiveMessage(async (message) => {
        switch (message.type) {
            case 'openFile':
                const startTime = Date.now();
                const viewColumn = message.viewColumn || vscode.ViewColumn.One;
                await openFileAtLine(message.path, message.line, viewColumn);
                const elapsed = Date.now() - startTime;
                // Send timing back to webview
                webview.postMessage({
                    type: 'fileOpened',
                    elapsed: elapsed,
                    path: message.path,
                    line: message.line,
                    viewColumn: viewColumn
                });
                break;
            case 'log':
                console.log('[LK Viewer]', message.message);
                break;
        }
    });
}
async function openFileAtLine(filePath, line, viewColumn) {
    try {
        const uri = vscode.Uri.file(filePath);
        const doc = await vscode.workspace.openTextDocument(uri);
        const lineNum = (line !== undefined && line >= 0) ? line : 0;
        const position = new vscode.Position(lineNum, 0);
        await vscode.window.showTextDocument(doc, {
            viewColumn: viewColumn || vscode.ViewColumn.One,
            preview: false,
            selection: new vscode.Range(position, position)
        });
        // Reveal the line
        const editor = vscode.window.activeTextEditor;
        if (editor) {
            editor.revealRange(new vscode.Range(position, position), vscode.TextEditorRevealType.InCenter);
        }
    }
    catch (error) {
        vscode.window.showErrorMessage(`Failed to open file: ${error}`);
    }
}
function getWebviewContent(webview, extensionUri) {
    return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>LK Viewer with React Flow</title>
    
    <!-- React and React DOM from CDN -->
    <script crossorigin src="https://unpkg.com/react@18/umd/react.production.min.js"></script>
    <script crossorigin src="https://unpkg.com/react-dom@18/umd/react-dom.production.min.js"></script>
    
    <!-- React Flow from CDN -->
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/@xyflow/react@12/dist/style.css">
    <script src="https://cdn.jsdelivr.net/npm/@xyflow/react@12/dist/umd/index.min.js"></script>
    
    <style>
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }
        html, body, #root {
            width: 100%;
            height: 100%;
            overflow: hidden;
        }
        body {
            font-family: var(--vscode-font-family);
            color: var(--vscode-foreground);
            background-color: var(--vscode-editor-background);
        }
        
        /* React Flow styling */
        .react-flow {
            background: var(--vscode-editor-background) !important;
        }
        .react-flow__node {
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: 1px solid var(--vscode-panel-border);
            border-radius: 6px;
            padding: 10px 15px;
            font-size: 12px;
        }
        .react-flow__node.selected {
            border: 2px solid var(--vscode-focusBorder);
            box-shadow: 0 0 0 2px var(--vscode-focusBorder);
        }
        .react-flow__edge-path {
            stroke: var(--vscode-foreground);
            stroke-width: 2;
        }
        .react-flow__edge.selected .react-flow__edge-path {
            stroke: var(--vscode-focusBorder);
            stroke-width: 3;
        }
        .react-flow__controls {
            background: var(--vscode-editor-background);
            border: 1px solid var(--vscode-panel-border);
        }
        .react-flow__controls-button {
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border-bottom: 1px solid var(--vscode-panel-border);
        }
        .react-flow__controls-button:hover {
            background: var(--vscode-button-hoverBackground);
        }
        
        /* Floating controls panel */
        .floating-controls {
            position: absolute;
            top: 10px;
            left: 10px;
            right: 10px;
            z-index: 5;
            background: var(--vscode-editor-background);
            padding: 12px;
            border-radius: 8px;
            border: 1px solid var(--vscode-panel-border);
            box-shadow: 0 4px 12px rgba(0,0,0,0.4);
            max-height: 80vh;
            overflow-y: auto;
        }
        .floating-header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            margin-bottom: 10px;
        }
        .floating-header h3 {
            margin: 0;
            font-size: 14px;
            font-weight: 600;
        }
        .collapse-btn {
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            border-radius: 4px;
            padding: 4px 10px;
            cursor: pointer;
            font-size: 12px;
            font-weight: 500;
        }
        .collapse-btn:hover {
            background: var(--vscode-button-hoverBackground);
        }
        .controls-content {
            display: block;
        }
        .controls-content.collapsed {
            display: none;
        }
        .file-item {
            margin: 8px 0;
            padding: 8px;
            background: var(--vscode-editor-inactiveSelectionBackground);
            border-radius: 4px;
        }
        .file-item-title {
            font-weight: 600;
            margin-bottom: 6px;
            font-size: 12px;
        }
        .open-buttons {
            display: flex;
            gap: 4px;
        }
        .open-btn {
            flex: 1;
            padding: 5px 8px;
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            border-radius: 3px;
            cursor: pointer;
            font-size: 10px;
            font-weight: 500;
            transition: all 0.15s;
        }
        .open-btn:hover {
            background: var(--vscode-button-hoverBackground);
            transform: translateY(-1px);
        }
        .timing {
            margin-top: 10px;
            padding: 8px;
            background: var(--vscode-editor-inactiveSelectionBackground);
            border-radius: 4px;
            font-size: 11px;
            font-family: monospace;
        }
        .timing.success {
            background: var(--vscode-testing-iconPassed);
            color: white;
        }
    </style>
</head>
<body>
    <div id="root"></div>

    <script>
        const { useState, useCallback } = React;
        const { ReactFlow, Background, Controls, MiniMap } = ReactFlowRenderer;
        const vscode = acquireVsCodeApi();
        
        // Initial nodes and edges
        const initialNodes = [
            {
                id: '1',
                type: 'default',
                data: { label: 'System' },
                position: { x: 250, y: 25 },
            },
            {
                id: '2',
                type: 'default',
                data: { label: 'Robot' },
                position: { x: 100, y: 125 },
            },
            {
                id: '3',
                type: 'default',
                data: { label: 'Sensor' },
                position: { x: 400, y: 125 },
            },
            {
                id: '4',
                type: 'default',
                data: { label: 'Controller' },
                position: { x: 250, y: 225 },
            },
        ];

        const initialEdges = [
            { id: 'e1-2', source: '1', target: '2', animated: true },
            { id: 'e1-3', source: '1', target: '3', animated: true },
            { id: 'e2-4', source: '2', target: '4' },
            { id: 'e3-4', source: '3', target: '4' },
        ];

        function App() {
            const [nodes, setNodes] = useState(initialNodes);
            const [edges, setEdges] = useState(initialEdges);
            const [collapsed, setCollapsed] = useState(false);
            const [timing, setTiming] = useState(null);

            const onNodesChange = useCallback((changes) => {
                setNodes((nds) => ReactFlowRenderer.applyNodeChanges(changes, nds));
            }, []);

            const onEdgesChange = useCallback((changes) => {
                setEdges((eds) => ReactFlowRenderer.applyEdgeChanges(changes, eds));
            }, []);

            const onConnect = useCallback((connection) => {
                setEdges((eds) => ReactFlowRenderer.addEdge(connection, eds));
            }, []);

            const openFile = (path, line, viewColumn) => {
                vscode.postMessage({
                    type: 'openFile',
                    path: path,
                    line: line,
                    viewColumn: viewColumn
                });
                setTiming({ status: 'pending', path, line, viewColumn });
            };

            // Listen for messages from extension
            React.useEffect(() => {
                window.addEventListener('message', event => {
                    const message = event.data;
                    if (message.type === 'fileOpened') {
                        setTiming({
                            status: 'success',
                            ...message
                        });
                        setTimeout(() => setTiming(null), 3000);
                    }
                });
            }, []);

            return React.createElement('div', { style: { width: '100%', height: '100%', position: 'relative' } },
                React.createElement(ReactFlow, {
                    nodes: nodes,
                    edges: edges,
                    onNodesChange: onNodesChange,
                    onEdgesChange: onEdgesChange,
                    onConnect: onConnect,
                    fitView: true
                },
                    React.createElement(Background),
                    React.createElement(Controls),
                    React.createElement(MiniMap, { 
                        nodeColor: 'var(--vscode-button-background)',
                        maskColor: 'rgba(0, 0, 0, 0.5)'
                    })
                ),
                
                // Floating controls
                React.createElement('div', { className: 'floating-controls' },
                    React.createElement('div', { className: 'floating-header' },
                        React.createElement('h3', null, '⚡ LK File Navigator'),
                        React.createElement('button', { 
                            className: 'collapse-btn',
                            onClick: () => setCollapsed(!collapsed)
                        }, collapsed ? '▼ Show' : '▲ Hide')
                    ),
                    
                    React.createElement('div', { className: collapsed ? 'controls-content collapsed' : 'controls-content' },
                        timing && React.createElement('div', { 
                            className: timing.status === 'success' ? 'timing success' : 'timing'
                        }, timing.status === 'pending' ? 
                            \`⏳ Opening file...\` :
                            \`✅ Opened in \${timing.viewColumn === 1 ? 'Left' : timing.viewColumn === 2 ? 'Right' : 'Active'} Panel | \${timing.elapsed}ms\`
                        ),
                        
                        // File items
                        [
                            { title: '📄 features.md:20', path: '/home/bam/bam_ws/src/like_keras/lk_viewer/features.md', line: 20 },
                            { title: '📄 README.md:79', path: '/home/bam/bam_ws/src/like_keras/lk_viewer/README.md', line: 79 },
                            { title: '🐍 diagnostics.py:30', path: '/home/bam/bam_ws/src/like_keras/lk/msgs/diagnostics.py', line: 30 }
                        ].map((file, i) => 
                            React.createElement('div', { key: i, className: 'file-item' },
                                React.createElement('div', { className: 'file-item-title' }, file.title),
                                React.createElement('div', { className: 'open-buttons' },
                                    React.createElement('button', { 
                                        className: 'open-btn',
                                        onClick: () => openFile(file.path, file.line, 1)
                                    }, '← Left'),
                                    React.createElement('button', { 
                                        className: 'open-btn',
                                        onClick: () => openFile(file.path, file.line, 2)
                                    }, 'Right →'),
                                    React.createElement('button', { 
                                        className: 'open-btn',
                                        onClick: () => openFile(file.path, file.line, -1)
                                    }, '📍 Here')
                                )
                            )
                        )
                    )
                )
            );
        }

        // Render the app
        const root = ReactDOM.createRoot(document.getElementById('root'));
        root.render(React.createElement(App));
    </script>
</body>
</html>`;
}
function deactivate() { }
//# sourceMappingURL=extension_reactflow.js.map