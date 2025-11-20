// extension.ts - Add file watcher for auto-reload

import * as vscode from 'vscode';
import * as path from 'path';
import { spawn, ChildProcess } from 'child_process';

let pythonProcess: ChildProcess | null = null;
let currentSystemPanel: vscode.WebviewPanel | null = null;
let currentSystemFile: string | null = null;
let currentSystemClass: string | null = null;

export function activate(context: vscode.ExtensionContext) {
    console.log('[Extension] LK Viewer activating...');
    
    // Start Python backend
    startPythonBackend(context);
    
    // Register right-click command
    context.subscriptions.push(
        vscode.commands.registerCommand('lk-viewer.visualizeSystem', async () => {
            await visualizeSystemAtCursor(context);
        })
    );
    
    // Watch for file changes and auto-reload
    const fileWatcher = vscode.workspace.createFileSystemWatcher('**/*.py');
    
    fileWatcher.onDidChange(async (uri) => {
        // If the changed file is the current system file, reload
        if (currentSystemPanel && currentSystemFile === uri.fsPath) {
            console.log(`[Extension] File ${uri.fsPath} changed, reloading system...`);
            await reloadCurrentSystem();
        }
    });
    
    context.subscriptions.push(fileWatcher);
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
    const className = editor.document.getText(wordRange);
    
    const filePath = editor.document.uri.fsPath;
    const line = position.line;
    
    console.log(`[Extension] Visualizing ${className} from ${filePath}:${line}`);
    
    // Store current system info for reload
    currentSystemFile = filePath;
    currentSystemClass = className;
    
    // Call Python backend to trace system
    try {
        const response = await fetch('http://localhost:8765/trace_system', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                filePath: filePath,
                className: className,
                line: line
            })
        });
        
        const result = await response.json();
        
        if (result.error) {
            vscode.window.showErrorMessage(`Error tracing system: ${result.error}`);
            return;
        }
        
        // Create or reuse panel
        if (currentSystemPanel) {
            currentSystemPanel.title = `System: ${className}`;
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
            
            currentSystemPanel.webview.html = getWebviewContent(currentSystemPanel.webview, context.extensionUri);
            
            currentSystemPanel.onDidDispose(() => {
                currentSystemPanel = null;
                currentSystemFile = null;
                currentSystemClass = null;
            });
            
            // Handle messages from webview
            setupSystemWebviewMessaging(currentSystemPanel.webview);
        }
        
        // Send graph to webview
        currentSystemPanel.webview.postMessage({
            type: 'loadSystem',
            graph: result.graph,
            systemClass: className,
            filePath: filePath,
            componentsCount: result.components_count
        });
        
        vscode.window.showInformationMessage(
            `Visualized ${className} with ${result.components_count} components`
        );
        
    } catch (error: any) {
        vscode.window.showErrorMessage(`Failed to trace system: ${error.message}`);
    }
}

async function reloadCurrentSystem() {
    if (!currentSystemFile || !currentSystemClass || !currentSystemPanel) {
        return;
    }
    
    console.log(`[Extension] Reloading system ${currentSystemClass}`);
    
    try {
        const response = await fetch('http://localhost:8765/trace_system', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                filePath: currentSystemFile,
                className: currentSystemClass,
                line: 0
            })
        });
        
        const result = await response.json();
        
        if (!result.error) {
            currentSystemPanel.webview.postMessage({
                type: 'loadSystem',
                graph: result.graph,
                systemClass: currentSystemClass,
                filePath: currentSystemFile,
                componentsCount: result.components_count
            });
            
            // Show subtle notification
            vscode.window.setStatusBarMessage(
                `✅ System reloaded: ${result.components_count} components`,
                3000
            );
        }
    } catch (error) {
        console.error('[Extension] Error reloading system:', error);
    }
}

function setupSystemWebviewMessaging(webview: vscode.Webview) {
    webview.onDidReceiveMessage(async (message) => {
        if (message.type === 'openFile') {
            await openFileAtLine(message.path, message.line, message.viewColumn || vscode.ViewColumn.One);
        } else if (message.type === 'findDefinition') {
            // Use VS Code's built-in "Go to Definition"
            await findComponentDefinition(message.componentType);
        }
    });
}

async function findComponentDefinition(componentType: string) {
    // Use VS Code's workspace symbol search
    const symbols = await vscode.commands.executeCommand<vscode.SymbolInformation[]>(
        'vscode.executeWorkspaceSymbolProvider',
        componentType
    );
    
    if (symbols && symbols.length > 0) {
        const symbol = symbols[0];
        const doc = await vscode.workspace.openTextDocument(symbol.location.uri);
        await vscode.window.showTextDocument(doc, {
            selection: symbol.location.range,
            viewColumn: vscode.ViewColumn.One
        });
    } else {
        vscode.window.showInformationMessage(`Definition for ${componentType} not found`);
    }
}

// ... rest of extension code


