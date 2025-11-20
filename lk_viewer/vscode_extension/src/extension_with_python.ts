import * as vscode from 'vscode';
import * as path from 'path';
import { spawn, ChildProcess } from 'child_process';

let pythonProcess: ChildProcess | null = null;

export function activate(context: vscode.ExtensionContext) {
    console.log('[Extension] LK Viewer activating...');
    
    // Start Python backend server
    startPythonBackend(context);
    
    const provider = new LKViewerWebviewProvider(context.extensionUri);
    context.subscriptions.push(
        vscode.window.registerWebviewViewProvider('lk-viewer.panel', provider)
    );

    context.subscriptions.push(
        vscode.commands.registerCommand('lk-viewer.openPanel', () => {
            console.log('[Extension] Opening panel...');
            const panel = vscode.window.createWebviewPanel(
                'lkViewer',
                'LK Viewer',
                vscode.ViewColumn.Two,
                { enableScripts: true, retainContextWhenHidden: true }
            );
            panel.webview.html = getWebviewContent(panel.webview, context.extensionUri);
            setupMessageHandling(panel.webview);
        })
    );
    
    // Auto-open the panel on activation
    setTimeout(() => {
        console.log('[Extension] Auto-opening LK Viewer panel...');
        vscode.commands.executeCommand('lk-viewer.openPanel');
    }, 500);
}

function startPythonBackend(context: vscode.ExtensionContext) {
    // Path to your Python backend script
    const pythonScriptPath = path.join(context.extensionPath, 'python_backend', 'server.py');
    
    // Get workspace root (where your Python code is)
    const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath || '';
    
    console.log('[Extension] Starting Python backend...');
    
    // Spawn Python process
    pythonProcess = spawn('python3', [pythonScriptPath, workspaceRoot], {
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

class LKViewerWebviewProvider implements vscode.WebviewViewProvider {
    constructor(private readonly _extensionUri: vscode.Uri) {}
    
    public resolveWebviewView(webviewView: vscode.WebviewView) {
        webviewView.webview.options = { enableScripts: true };
        webviewView.webview.html = getWebviewContent(webviewView.webview, this._extensionUri);
        setupMessageHandling(webviewView.webview);
    }
}

function setupMessageHandling(webview: vscode.Webview) {
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

async function openFileAtLine(filePath: string, line?: number, viewColumn?: vscode.ViewColumn) {
    try {
        const doc = await vscode.workspace.openTextDocument(vscode.Uri.file(filePath));
        const lineNum = (line !== undefined && line >= 0) ? line : 0;
        const position = new vscode.Position(lineNum, 0);
        await vscode.window.showTextDocument(doc, {
            viewColumn: viewColumn || vscode.ViewColumn.One,
            preview: false,
            selection: new vscode.Range(position, position)
        });
        vscode.window.activeTextEditor?.revealRange(
            new vscode.Range(position, position),
            vscode.TextEditorRevealType.InCenter
        );
    } catch (error) {
        vscode.window.showErrorMessage(`Failed to open file: ${error}`);
    }
}

function getWebviewContent(webview: vscode.Webview, extensionUri: vscode.Uri) {
    const isProduction = true;
    const localServerUrl = 'http://localhost:5173';
    
    let scriptUri: string;
    let styleUri: string | null = null;
    
    if (isProduction) {
        scriptUri = webview.asWebviewUri(
            vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.js')
        ).toString();
        styleUri = webview.asWebviewUri(
            vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.css')
        ).toString();
    } else {
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

export function deactivate() {
    // Kill Python process when extension deactivates
    if (pythonProcess) {
        console.log('[Extension] Stopping Python backend...');
        pythonProcess.kill();
        pythonProcess = null;
    }
}


