import * as vscode from 'vscode';
import * as path from 'path';
import { exec } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

/**
 * Custom Editor for Python System Files
 * Provides WYSIWYG editing: Python code ↔ Graph visualization
 */
export class PythonSystemEditorProvider implements vscode.CustomTextEditorProvider {
    
    public static register(context: vscode.ExtensionContext): vscode.Disposable {
        const provider = new PythonSystemEditorProvider(context);
        
        return vscode.window.registerCustomEditorProvider(
            'lk-viewer.pythonSystemEditor',
            provider,
            {
                webviewOptions: {
                    retainContextWhenHidden: true,
                },
                supportsMultipleEditorsPerDocument: false,
            }
        );
    }
    
    constructor(private readonly context: vscode.ExtensionContext) {}
    
    /**
     * Called when custom editor is opened
     */
    public async resolveCustomTextEditor(
        document: vscode.TextDocument,
        webviewPanel: vscode.WebviewPanel,
        _token: vscode.CancellationToken
    ): Promise<void> {
        
        // Setup webview
        webviewPanel.webview.options = {
            enableScripts: true,
        };
        webviewPanel.webview.html = this.getHtmlForWebview(webviewPanel.webview);
        
        // Send initial document to webview
        await this.updateWebview(document, webviewPanel.webview);
        
        // Listen for changes from VS Code (user edited code directly)
        const changeDocumentSubscription = vscode.workspace.onDidChangeTextDocument(e => {
            if (e.document.uri.toString() === document.uri.toString()) {
                this.updateWebview(document, webviewPanel.webview);
            }
        });
        
        // Listen for changes from webview (user edited graph)
        const messageSubscription = webviewPanel.webview.onDidReceiveMessage(async message => {
            switch (message.type) {
                case 'edit':
                    // User edited the graph - update the document!
                    await this.updateTextDocument(document, message.edits);
                    break;
                    
                case 'openFile':
                    // Open file at specific line
                    await this.openFileAtLine(message.path, message.line);
                    break;
            }
        });
        
        // Cleanup
        webviewPanel.onDidDispose(() => {
            changeDocumentSubscription.dispose();
            messageSubscription.dispose();
        });
    }
    
    /**
     * Parse document and send graph to webview
     */
    private async updateWebview(document: vscode.TextDocument, webview: vscode.Webview) {
        const text = document.getText();
        
        // Call Python to parse
        const pythonScript = path.join(
            this.context.extensionPath,
            'python_backend',
            'system_tracer_cli.py'
        );
        
        try {
            // Find System classes in the document
            const className = this.extractSystemClassName(text);
            if (!className) {
                webview.postMessage({
                    type: 'error',
                    message: 'No System class found in document'
                });
                return;
            }
            
            const { stdout } = await execAsync(
                `python3 "${pythonScript}" "${document.uri.fsPath}" "${className}" ""`
            );
            
            const result = JSON.parse(stdout);
            
            // Send graph + original text to webview
            webview.postMessage({
                type: 'update',
                graph: result.graph,
                sourceCode: text,
                filePath: document.uri.fsPath,
                className: className
            });
            
        } catch (error: any) {
            console.error('Error parsing document:', error);
            webview.postMessage({
                type: 'error',
                message: error.message
            });
        }
    }
    
    /**
     * Update the text document based on graph edits
     */
    private async updateTextDocument(
        document: vscode.TextDocument,
        edits: Array<{line: number, newText: string}>
    ): Promise<void> {
        
        const edit = new vscode.WorkspaceEdit();
        
        // Apply edits to document
        for (const { line, newText } of edits) {
            const lineRange = document.lineAt(line).range;
            edit.replace(document.uri, lineRange, newText);
        }
        
        // This triggers undo/redo, save state, etc.
        await vscode.workspace.applyEdit(edit);
    }
    
    private extractSystemClassName(text: string): string | null {
        // Simple regex to find "class XxxSystem:"
        const match = text.match(/class\s+(\w*System)\s*:/);
        return match ? match[1] : null;
    }
    
    private async openFileAtLine(filePath: string, line: number) {
        const doc = await vscode.workspace.openTextDocument(vscode.Uri.file(filePath));
        const position = new vscode.Position(line, 0);
        await vscode.window.showTextDocument(doc, {
            selection: new vscode.Range(position, position)
        });
    }
    
    private getHtmlForWebview(webview: vscode.Webview): string {
        const scriptUri = webview.asWebviewUri(
            vscode.Uri.joinPath(this.context.extensionUri, 'webview-ui', 'build', 'assets', 'index.js')
        );
        const styleUri = webview.asWebviewUri(
            vscode.Uri.joinPath(this.context.extensionUri, 'webview-ui', 'build', 'assets', 'index.css')
        );
        
        const nonce = this.getNonce();
        
        return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${webview.cspSource} 'unsafe-inline'; script-src 'nonce-${nonce}';">
    <link href="${styleUri}" rel="stylesheet">
    <title>System Editor</title>
</head>
<body>
    <div id="root"></div>
    <script type="text/javascript" nonce="${nonce}" src="${scriptUri}"></script>
</body>
</html>`;
    }
    
    private getNonce() {
        let text = '';
        const possible = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
        for (let i = 0; i < 32; i++) {
            text += possible.charAt(Math.floor(Math.random() * possible.length));
        }
        return text;
    }
}

// Register in extension.ts:
export function activate(context: vscode.ExtensionContext) {
    context.subscriptions.push(
        PythonSystemEditorProvider.register(context)
    );
}


