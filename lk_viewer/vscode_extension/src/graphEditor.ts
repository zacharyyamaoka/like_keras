import * as vscode from 'vscode';

/**
 * LK System Graph Editor - Bidirectional editing like Mark Sharp!
 * 
 * Visual Graph <-> Python Code
 * 
 * Edit the graph visually -> Updates Python code
 * Edit Python code -> Updates graph visualization
 */
export function activate(context: vscode.ExtensionContext) {
    context.subscriptions.push(
        LKGraphEditorProvider.register(context)
    );
}

class LKGraphEditorProvider implements vscode.CustomTextEditorProvider {
    
    public static register(context: vscode.ExtensionContext): vscode.Disposable {
        const provider = new LKGraphEditorProvider(context);
        const providerRegistration = vscode.window.registerCustomEditorProvider(
            'lk-viewer.graphEditor',
            provider,
            {
                webviewOptions: {
                    retainContextWhenHidden: true,
                },
                supportsMultipleEditorsPerDocument: false,
            }
        );
        return providerRegistration;
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
        
        webviewPanel.webview.options = {
            enableScripts: true,
        };

        webviewPanel.webview.html = this.getHtmlForWebview(webviewPanel.webview);

        // TEXT -> VISUAL: When Python code changes, update graph
        const changeDocumentSubscription = vscode.workspace.onDidChangeTextDocument(e => {
            if (e.document.uri.toString() === document.uri.toString()) {
                // Parse the Python code and send graph data to webview
                this.updateWebview(webviewPanel.webview, document);
            }
        });

        // VISUAL -> TEXT: When graph changes, update Python code
        webviewPanel.webview.onDidReceiveMessage(async message => {
            switch (message.type) {
                case 'updateCode':
                    // Apply edits to the document
                    await this.updatePythonCode(document, message.changes);
                    break;
                    
                case 'nodeAdded':
                    await this.addComponent(document, message.component);
                    break;
                    
                case 'nodeMoved':
                    await this.updateComponentPosition(document, message.nodeId, message.position);
                    break;
                    
                case 'edgeAdded':
                    await this.addConnection(document, message.from, message.to);
                    break;
            }
        });

        // Send initial graph data to webview
        this.updateWebview(webviewPanel.webview, document);

        // Cleanup
        webviewPanel.onDidDispose(() => {
            changeDocumentSubscription.dispose();
        });
    }

    /**
     * TEXT -> VISUAL: Parse Python and send to graph
     */
    private updateWebview(webview: vscode.Webview, document: vscode.TextDocument) {
        const code = document.getText();
        
        // Parse your Python code to extract graph structure
        const graphData = this.parsePythonToGraph(code);
        
        webview.postMessage({
            type: 'updateGraph',
            graphData: graphData
        });
    }

    /**
     * VISUAL -> TEXT: Update Python code from graph changes
     */
    private async updatePythonCode(
        document: vscode.TextDocument,
        changes: any
    ): Promise<void> {
        const edit = new vscode.WorkspaceEdit();
        
        // Example: Update component definition
        // Find the line with the component and replace it
        const text = document.getText();
        const lines = text.split('\n');
        
        for (const change of changes) {
            // Find where to apply the edit
            const lineIndex = this.findComponentLine(lines, change.componentId);
            if (lineIndex >= 0) {
                const line = document.lineAt(lineIndex);
                const newText = this.generateComponentCode(change.data);
                
                edit.replace(
                    document.uri,
                    line.range,
                    newText
                );
            }
        }
        
        await vscode.workspace.applyEdit(edit);
    }

    /**
     * Add a new component to Python code
     */
    private async addComponent(document: vscode.TextDocument, component: any) {
        const edit = new vscode.WorkspaceEdit();
        
        // Generate Python code for new component
        const componentCode = `
${component.name} = Component(
    name="${component.name}",
    position=(${component.x}, ${component.y})
)
`;
        
        // Find insertion point (e.g., after imports)
        const insertPosition = this.findInsertionPoint(document);
        edit.insert(document.uri, insertPosition, componentCode);
        
        await vscode.workspace.applyEdit(edit);
    }

    /**
     * Update component position in Python code
     */
    private async updateComponentPosition(
        document: vscode.TextDocument,
        nodeId: string,
        position: { x: number; y: number }
    ) {
        const edit = new vscode.WorkspaceEdit();
        const text = document.getText();
        
        // Find and update the position parameter
        // This is simplified - you'd use proper AST parsing
        const regex = new RegExp(`${nodeId}.*position=\\([^)]+\\)`, 'g');
        const match = regex.exec(text);
        
        if (match) {
            const start = document.positionAt(match.index);
            const end = document.positionAt(match.index + match[0].length);
            const newText = `${nodeId} = Component(position=(${position.x}, ${position.y}))`;
            
            edit.replace(document.uri, new vscode.Range(start, end), newText);
            await vscode.workspace.applyEdit(edit);
        }
    }

    /**
     * Add a connection between components
     */
    private async addConnection(document: vscode.TextDocument, from: string, to: string) {
        const edit = new vscode.WorkspaceEdit();
        
        // Generate Python code for new connection
        const connectionCode = `\n${from}.connect(${to})\n`;
        
        // Find insertion point (end of file)
        const lastLine = document.lineAt(document.lineCount - 1);
        const insertPosition = lastLine.range.end;
        
        edit.insert(document.uri, insertPosition, connectionCode);
        await vscode.workspace.applyEdit(edit);
    }

    /**
     * Parse Python code to graph data structure
     */
    private parsePythonToGraph(code: string): any {
        // TODO: Use actual Python AST parsing
        // For now, simplified parsing
        
        const nodes: any[] = [];
        const edges: any[] = [];
        
        // Parse component definitions
        const componentRegex = /(\w+)\s*=\s*Component\(([^)]+)\)/g;
        let match;
        
        while ((match = componentRegex.exec(code)) !== null) {
            const name = match[1];
            const params = match[2];
            
            // Extract position if available
            const posMatch = /position=\(([^,]+),\s*([^)]+)\)/.exec(params);
            
            nodes.push({
                id: name,
                label: name,
                x: posMatch ? parseFloat(posMatch[1]) : 0,
                y: posMatch ? parseFloat(posMatch[2]) : 0,
            });
        }
        
        // Parse connections (ports, edges, etc.)
        const connectionRegex = /(\w+)\.connect\((\w+)\)/g;
        while ((match = connectionRegex.exec(code)) !== null) {
            edges.push({
                from: match[1],
                to: match[2]
            });
        }
        
        return { nodes, edges };
    }

    private findComponentLine(lines: string[], componentId: string): number {
        return lines.findIndex(line => line.includes(`${componentId} =`));
    }

    private findInsertionPoint(document: vscode.TextDocument): vscode.Position {
        // Find end of imports, or beginning of file
        const text = document.getText();
        const lines = text.split('\n');
        
        let lastImportLine = 0;
        for (let i = 0; i < lines.length; i++) {
            if (lines[i].startsWith('import ') || lines[i].startsWith('from ')) {
                lastImportLine = i;
            }
        }
        
        return new vscode.Position(lastImportLine + 2, 0);
    }

    private generateComponentCode(data: any): string {
        // Generate Python code from component data
        return `${data.name} = Component(name="${data.name}", position=(${data.x}, ${data.y}))`;
    }

    private getHtmlForWebview(webview: vscode.Webview): string {
        return `<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <style>
        body { margin: 0; padding: 0; overflow: hidden; }
        #graph-container { width: 100vw; height: 100vh; }
    </style>
</head>
<body>
    <div id="graph-container"></div>
    
    <script>
        const vscode = acquireVsCodeApi();
        
        // Listen for graph updates from Python code changes
        window.addEventListener('message', event => {
            const message = event.data;
            
            if (message.type === 'updateGraph') {
                // Update your React Flow / graph visualization
                renderGraph(message.graphData);
            }
        });
        
        // Send changes back to Python when graph is edited
        function onNodeMoved(nodeId, position) {
            vscode.postMessage({
                type: 'nodeMoved',
                nodeId: nodeId,
                position: position
            });
        }
        
        function onNodeAdded(component) {
            vscode.postMessage({
                type: 'nodeAdded',
                component: component
            });
        }
        
        function onEdgeAdded(from, to) {
            vscode.postMessage({
                type: 'edgeAdded',
                from: from,
                to: to
            });
        }
        
        function renderGraph(graphData) {
            // Render with React Flow, D3, etc.
            console.log('Rendering graph:', graphData);
        }
    </script>
</body>
</html>`;
    }
}

export function deactivate() {}

