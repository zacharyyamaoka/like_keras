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
exports.PythonSystemEditorProvider = void 0;
exports.activate = activate;
const vscode = __importStar(require("vscode"));
const path = __importStar(require("path"));
const child_process_1 = require("child_process");
const util_1 = require("util");
const execAsync = (0, util_1.promisify)(child_process_1.exec);
/**
 * Custom Editor for Python System Files
 * Provides WYSIWYG editing: Python code ↔ Graph visualization
 */
class PythonSystemEditorProvider {
    static register(context) {
        const provider = new PythonSystemEditorProvider(context);
        return vscode.window.registerCustomEditorProvider('lk-viewer.pythonSystemEditor', provider, {
            webviewOptions: {
                retainContextWhenHidden: true,
            },
            supportsMultipleEditorsPerDocument: false,
        });
    }
    constructor(context) {
        this.context = context;
    }
    /**
     * Called when custom editor is opened
     */
    async resolveCustomTextEditor(document, webviewPanel, _token) {
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
        const messageSubscription = webviewPanel.webview.onDidReceiveMessage(async (message) => {
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
    async updateWebview(document, webview) {
        const text = document.getText();
        // Call Python to parse
        const pythonScript = path.join(this.context.extensionPath, 'python_backend', 'system_tracer_cli.py');
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
            const { stdout } = await execAsync(`python3 "${pythonScript}" "${document.uri.fsPath}" "${className}" ""`);
            const result = JSON.parse(stdout);
            // Send graph + original text to webview
            webview.postMessage({
                type: 'update',
                graph: result.graph,
                sourceCode: text,
                filePath: document.uri.fsPath,
                className: className
            });
        }
        catch (error) {
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
    async updateTextDocument(document, edits) {
        const edit = new vscode.WorkspaceEdit();
        // Apply edits to document
        for (const { line, newText } of edits) {
            const lineRange = document.lineAt(line).range;
            edit.replace(document.uri, lineRange, newText);
        }
        // This triggers undo/redo, save state, etc.
        await vscode.workspace.applyEdit(edit);
    }
    extractSystemClassName(text) {
        // Simple regex to find "class XxxSystem:"
        const match = text.match(/class\s+(\w*System)\s*:/);
        return match ? match[1] : null;
    }
    async openFileAtLine(filePath, line) {
        const doc = await vscode.workspace.openTextDocument(vscode.Uri.file(filePath));
        const position = new vscode.Position(line, 0);
        await vscode.window.showTextDocument(doc, {
            selection: new vscode.Range(position, position)
        });
    }
    getHtmlForWebview(webview) {
        const scriptUri = webview.asWebviewUri(vscode.Uri.joinPath(this.context.extensionUri, 'webview-ui', 'build', 'assets', 'index.js'));
        const styleUri = webview.asWebviewUri(vscode.Uri.joinPath(this.context.extensionUri, 'webview-ui', 'build', 'assets', 'index.css'));
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
    getNonce() {
        let text = '';
        const possible = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
        for (let i = 0; i < 32; i++) {
            text += possible.charAt(Math.floor(Math.random() * possible.length));
        }
        return text;
    }
}
exports.PythonSystemEditorProvider = PythonSystemEditorProvider;
// Register in extension.ts:
function activate(context) {
    context.subscriptions.push(PythonSystemEditorProvider.register(context));
}
//# sourceMappingURL=pythonSystemEditor.js.map