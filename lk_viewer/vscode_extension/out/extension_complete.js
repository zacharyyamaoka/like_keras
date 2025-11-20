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
const util_1 = require("util");
const execAsync = (0, util_1.promisify)(child_process_1.exec);
let currentSystemPanel = null;
let currentSystemFile = null;
let currentSystemClass = null;
function activate(context) {
    console.log('[Extension] LK Viewer activating...');
    // Register command: Right-click → Visualize System
    context.subscriptions.push(vscode.commands.registerCommand('lk-viewer.visualizeSystem', async () => {
        await visualizeSystemAtCursor(context);
    }));
    // Watch for file changes and auto-reload
    const fileWatcher = vscode.workspace.createFileSystemWatcher('**/*.py');
    fileWatcher.onDidChange(async (uri) => {
        if (currentSystemPanel && currentSystemFile === uri.fsPath) {
            console.log(`[Extension] File changed, reloading system...`);
            await reloadCurrentSystem(context);
        }
    });
    context.subscriptions.push(fileWatcher);
    vscode.window.showInformationMessage('LK Viewer activated! Right-click a class to visualize.');
}
async function visualizeSystemAtCursor(context) {
    const editor = vscode.window.activeTextEditor;
    if (!editor) {
        vscode.window.showErrorMessage('No active editor');
        return;
    }
    // Get class name under cursor
    const position = editor.selection.active;
    const wordRange = editor.document.getWordRangeAtPosition(position);
    if (!wordRange) {
        vscode.window.showErrorMessage('No word under cursor');
        return;
    }
    const className = editor.document.getText(wordRange);
    const filePath = editor.document.uri.fsPath;
    const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath || '';
    console.log(`[Extension] Visualizing ${className} from ${filePath}`);
    // Store for auto-reload
    currentSystemFile = filePath;
    currentSystemClass = className;
    // Call Python backend to trace system
    try {
        const result = await callPythonTracer(context, filePath, className, workspaceRoot);
        if (result.error) {
            vscode.window.showErrorMessage(`Error: ${result.error}`);
            return;
        }
        // Show graph in webview
        showSystemGraph(context, result, className);
        vscode.window.showInformationMessage(`✅ Visualized ${className} with ${result.components_count} components`);
    }
    catch (error) {
        vscode.window.showErrorMessage(`Failed to trace system: ${error.message}`);
    }
}
async function callPythonTracer(context, filePath, className, workspaceRoot) {
    // Path to Python script
    const pythonScript = path.join(context.extensionPath, 'python_backend', 'system_tracer_cli.py');
    // Call Python directly
    const command = `python3 "${pythonScript}" "${filePath}" "${className}" "${workspaceRoot}"`;
    console.log(`[Extension] Running: ${command}`);
    const { stdout, stderr } = await execAsync(command);
    if (stderr) {
        console.error(`[Extension] Python stderr: ${stderr}`);
    }
    // Parse JSON output
    try {
        return JSON.parse(stdout);
    }
    catch (e) {
        throw new Error(`Failed to parse Python output: ${stdout}`);
    }
}
function showSystemGraph(context, result, className) {
    // Create or reuse webview panel
    if (currentSystemPanel) {
        currentSystemPanel.title = `System: ${className}`;
        currentSystemPanel.reveal(vscode.ViewColumn.Two);
    }
    else {
        currentSystemPanel = vscode.window.createWebviewPanel('lkSystemViewer', `System: ${className}`, vscode.ViewColumn.Two, {
            enableScripts: true,
            retainContextWhenHidden: true
        });
        currentSystemPanel.webview.html = getWebviewContent(currentSystemPanel.webview, context.extensionUri);
        // Handle file open requests from webview
        currentSystemPanel.webview.onDidReceiveMessage(async (message) => {
            if (message.type === 'openFile') {
                await openFileAtLine(message.path, message.line, message.viewColumn);
            }
        });
        currentSystemPanel.onDidDispose(() => {
            currentSystemPanel = null;
            currentSystemFile = null;
            currentSystemClass = null;
        });
    }
    // Send graph data to webview
    currentSystemPanel.webview.postMessage({
        type: 'loadSystem',
        graph: result.graph,
        systemClass: className,
        filePath: result.file_path,
        componentsCount: result.components_count
    });
}
async function reloadCurrentSystem(context) {
    if (!currentSystemFile || !currentSystemClass) {
        return;
    }
    const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath || '';
    try {
        const result = await callPythonTracer(context, currentSystemFile, currentSystemClass, workspaceRoot);
        if (currentSystemPanel && !result.error) {
            currentSystemPanel.webview.postMessage({
                type: 'loadSystem',
                graph: result.graph,
                systemClass: currentSystemClass,
                componentsCount: result.components_count
            });
            vscode.window.setStatusBarMessage(`✅ System reloaded: ${result.components_count} components`, 2000);
        }
    }
    catch (error) {
        console.error('[Extension] Error reloading:', error);
    }
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
    const scriptUri = webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.js'));
    const styleUri = webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.css'));
    const nonce = getNonce();
    return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${webview.cspSource} 'unsafe-inline'; script-src 'nonce-${nonce}';">
    <link href="${styleUri}" rel="stylesheet">
    <title>System Viewer</title>
</head>
<body>
    <div id="root"></div>
    <script type="text/javascript" nonce="${nonce}" src="${scriptUri}"></script>
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
    console.log('[Extension] Deactivating...');
}
//# sourceMappingURL=extension_complete.js.map