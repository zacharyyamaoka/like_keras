import * as vscode from 'vscode';

export function activate(context: vscode.ExtensionContext) {
    console.log('[Extension] LK Viewer activating...');
    vscode.window.showInformationMessage('LK Viewer activated!');

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
    const isProduction = true; // Set to false to use dev server
    const localServerUrl = 'http://localhost:5173';
    
    let scriptUri: string;
    let styleUri: string | null = null;
    
    if (isProduction) {
        // Production: load from built React app
        scriptUri = webview.asWebviewUri(
            vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.js')
        ).toString();
        styleUri = webview.asWebviewUri(
            vscode.Uri.joinPath(extensionUri, 'webview-ui', 'build', 'assets', 'index.css')
        ).toString();
    } else {
        // Development: load from Vite dev server
        // Run: cd webview-ui && npm run dev
        scriptUri = `${localServerUrl}/src/main.tsx`;
    }

    const nonce = getNonce();

    return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${webview.cspSource} 'unsafe-inline'; script-src 'nonce-${nonce}' ${isProduction ? '' : localServerUrl};">
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

export function deactivate() {}
