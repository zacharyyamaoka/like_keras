import * as vscode from 'vscode';
import * as path from 'path';
import { exec } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

let currentSystemPanel: vscode.WebviewPanel | null = null;
let currentSystemFile: string | null = null;
let currentSystemClass: string | null = null;

export function activate(context: vscode.ExtensionContext) {
    console.log('[Extension] LK Viewer activating...');
    
    // Register command: Right-click → Visualize System
    context.subscriptions.push(
        vscode.commands.registerCommand('lk-viewer.visualizeSystem', async () => {
            await visualizeSystemAtCursor(context);
        })
    );
    
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

async function visualizeSystemAtCursor(context: vscode.ExtensionContext) {
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
        
        vscode.window.showInformationMessage(
            `✅ Visualized ${className} with ${result.components_count} components`
        );
        
    } catch (error: any) {
        vscode.window.showErrorMessage(`Failed to trace system: ${error.message}`);
    }
}

async function callPythonTracer(
    context: vscode.ExtensionContext,
    filePath: string,
    className: string,
    workspaceRoot: string
): Promise<any> {
    // Path to Python script
    const pythonScript = path.join(
        context.extensionPath,
        'python_backend',
        'system_tracer_cli.py'
    );
    
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
    } catch (e) {
        throw new Error(`Failed to parse Python output: ${stdout}`);
    }
}

function showSystemGraph(
    context: vscode.ExtensionContext,
    result: any,
    className: string
) {
    // Create or reuse webview panel
    if (currentSystemPanel) {
        currentSystemPanel.title = `System: ${className}`;
        currentSystemPanel.reveal(vscode.ViewColumn.Two);
    } else {
        currentSystemPanel = vscode.window.createWebviewPanel(
            'lkSystemViewer',
            `System: ${className}`,
            vscode.ViewColumn.Two,
            {
                enableScripts: true,
                retainContextWhenHidden: true
            }
        );
        
        currentSystemPanel.webview.html = getWebviewContent(
            currentSystemPanel.webview,
            context.extensionUri
        );
        
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

async function reloadCurrentSystem(context: vscode.ExtensionContext) {
    if (!currentSystemFile || !currentSystemClass) {
        return;
    }
    
    const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath || '';
    
    try {
        const result = await callPythonTracer(
            context,
            currentSystemFile,
            currentSystemClass,
            workspaceRoot
        );
        
        if (currentSystemPanel && !result.error) {
            currentSystemPanel.webview.postMessage({
                type: 'loadSystem',
                graph: result.graph,
                systemClass: currentSystemClass,
                componentsCount: result.components_count
            });
            
            vscode.window.setStatusBarMessage(
                `✅ System reloaded: ${result.components_count} components`,
                2000
            );
        }
    } catch (error) {
        console.error('[Extension] Error reloading:', error);
    }
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

function getWebviewContent(webview: vscode.Webview, extensionUri: vscode.Uri): string {
    const scriptUri = webview.asWebviewUri(
        vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.js')
    );
    const styleUri = webview.asWebviewUri(
        vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.css')
    );
    
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

export function deactivate() {
    console.log('[Extension] Deactivating...');
}


