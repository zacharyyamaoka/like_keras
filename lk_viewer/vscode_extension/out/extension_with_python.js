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
const path = __importStar(require("path"));
const child_process_1 = require("child_process");
let pythonProcess = null;
function activate(context) {
    console.log('[Extension] LK Viewer activating...');
    // Start Python backend server
    startPythonBackend(context);
    const provider = new LKViewerWebviewProvider(context.extensionUri);
    context.subscriptions.push(vscode.window.registerWebviewViewProvider('lk-viewer.panel', provider));
    context.subscriptions.push(vscode.commands.registerCommand('lk-viewer.openPanel', () => {
        console.log('[Extension] Opening panel...');
        const panel = vscode.window.createWebviewPanel('lkViewer', 'LK Viewer', vscode.ViewColumn.Two, { enableScripts: true, retainContextWhenHidden: true });
        panel.webview.html = getWebviewContent(panel.webview, context.extensionUri);
        setupMessageHandling(panel.webview);
    }));
    // Auto-open the panel on activation
    setTimeout(() => {
        console.log('[Extension] Auto-opening LK Viewer panel...');
        vscode.commands.executeCommand('lk-viewer.openPanel');
    }, 500);
}
function startPythonBackend(context) {
    // Path to your Python backend script
    const pythonScriptPath = path.join(context.extensionPath, 'python_backend', 'server.py');
    // Get workspace root (where your Python code is)
    const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath || '';
    console.log('[Extension] Starting Python backend...');
    // Spawn Python process
    pythonProcess = (0, child_process_1.spawn)('python3', [pythonScriptPath, workspaceRoot], {
        cwd: path.dirname(pythonScriptPath)
    });
    pythonProcess.stdout?.on('data', (data) => {
        console.log(`[Python Backend] ${data.toString()}`);
    });
    pythonProcess.stderr?.on('data', (data) => {
        console.error(`[Python Backend Error] ${data.toString()}`);
    });
    pythonProcess.on('close', (code) => {
        console.log(`[Extension] Python backend exited with code ${code}`);
        pythonProcess = null;
    });
    vscode.window.showInformationMessage('LK Viewer Python backend started on http://localhost:8765');
}
class LKViewerWebviewProvider {
    constructor(_extensionUri) {
        this._extensionUri = _extensionUri;
    }
    resolveWebviewView(webviewView) {
        webviewView.webview.options = { enableScripts: true };
        webviewView.webview.html = getWebviewContent(webviewView.webview, this._extensionUri);
        setupMessageHandling(webviewView.webview);
    }
}
function setupMessageHandling(webview) {
    webview.onDidReceiveMessage(async (message) => {
        if (message.type === 'openFile') {
            const startTime = Date.now();
            await openFileAtLine(message.path, message.line, message.viewColumn || vscode.ViewColumn.One);
            webview.postMessage({
                type: 'fileOpened',
                elapsed: Date.now() - startTime,
                path: message.path,
                line: message.line,
                viewColumn: message.viewColumn
            });
        }
    });
}
async function openFileAtLine(filePath, line, viewColumn) {
    try {
        const doc = await vscode.workspace.openTextDocument(vscode.Uri.file(filePath));
        const lineNum = (line !== undefined && line >= 0) ? line : 0;
        const position = new vscode.Position(lineNum, 0);
        await vscode.window.showTextDocument(doc, {
            viewColumn: viewColumn || vscode.ViewColumn.One,
            preview: false,
            selection: new vscode.Range(position, position)
        });
        vscode.window.activeTextEditor?.revealRange(new vscode.Range(position, position), vscode.TextEditorRevealType.InCenter);
    }
    catch (error) {
        vscode.window.showErrorMessage(`Failed to open file: ${error}`);
    }
}
function getWebviewContent(webview, extensionUri) {
    const isProduction = true;
    const localServerUrl = 'http://localhost:5173';
    let scriptUri;
    let styleUri = null;
    if (isProduction) {
        scriptUri = webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.js')).toString();
        styleUri = webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.css')).toString();
    }
    else {
        scriptUri = `${localServerUrl}/src/main.tsx`;
    }
    const nonce = getNonce();
    return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="Content-Security-Policy" content="default-src 'none'; connect-src http://localhost:8765; style-src ${webview.cspSource} 'unsafe-inline'; script-src 'nonce-${nonce}' ${isProduction ? '' : localServerUrl};">
    ${isProduction && styleUri ? `<link href="${styleUri}" rel="stylesheet">` : ''}
    <title>LK Viewer</title>
</head>
<body>
    <div id="root"></div>
    <script type="${isProduction ? 'text/javascript' : 'module'}" nonce="${nonce}" src="${scriptUri}"></script>
</body>
</html>`;
}
function getNonce() {
    let text = '';
    const possible = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    for (let i = 0; i < 32; i++) {
        text += possible.charAt(Math.floor(Math.random() * possible.length));
    }
    return text;
}
function deactivate() {
    // Kill Python process when extension deactivates
    if (pythonProcess) {
        console.log('[Extension] Stopping Python backend...');
        pythonProcess.kill();
        pythonProcess = null;
    }
}
//# sourceMappingURL=extension_with_python.js.map