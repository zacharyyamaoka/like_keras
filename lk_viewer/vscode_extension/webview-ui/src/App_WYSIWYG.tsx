import React, { useCallback, useState, useEffect } from 'react';
import ReactFlow, {
  Node,
  Edge,
  Background,
  Controls,
  MiniMap,
  addEdge,
  Connection,
  useNodesState,
  useEdgesState,
  NodeMouseHandler,
} from 'reactflow';
import 'reactflow/dist/style.css';
import './App.css';

declare const acquireVsCodeApi: any;
const vscode = acquireVsCodeApi();

interface NodeData {
  label: string;
  filePath?: string;
  line?: number;
  componentName?: string;
  componentType?: string;
  // For editing:
  sourceCode?: string;
}

function App() {
  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);
  const [selectedNode, setSelectedNode] = useState<Node | null>(null);
  const [editingNodeId, setEditingNodeId] = useState<string | null>(null);
  const [editValue, setEditValue] = useState('');

  const onConnect = useCallback(
    (params: Connection) => setEdges((eds) => addEdge(params, eds)),
    [setEdges]
  );

  // Single click: select, Double click: edit
  const onNodeClick: NodeMouseHandler = useCallback((event, node) => {
    setSelectedNode(node);
    
    // Jump to code line
    const nodeData = node.data as NodeData;
    if (nodeData.filePath && nodeData.line !== undefined) {
      vscode.postMessage({
        type: 'openFile',
        path: nodeData.filePath,
        line: nodeData.line
      });
    }
  }, []);

  const onNodeDoubleClick: NodeMouseHandler = useCallback((event, node) => {
    // Enter edit mode
    setEditingNodeId(node.id);
    const nodeData = node.data as NodeData;
    setEditValue(nodeData.componentName || nodeData.label);
  }, []);

  // Listen for updates from VS Code
  useEffect(() => {
    const handleMessage = (event: MessageEvent) => {
      const msg = event.data;
      
      switch (msg.type) {
        case 'update':
          // Document changed in VS Code - update graph
          setNodes(msg.graph.nodes);
          setEdges(msg.graph.edges);
          break;
          
        case 'error':
          console.error('Error from extension:', msg.message);
          break;
      }
    };

    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, [setNodes, setEdges]);

  // Handle node name edit
  const handleNodeEdit = (nodeId: string, newName: string) => {
    const node = nodes.find(n => n.id === nodeId);
    if (!node) return;
    
    const nodeData = node.data as NodeData;
    
    // Send edit to VS Code to update the Python code
    vscode.postMessage({
      type: 'edit',
      edits: [{
        line: nodeData.line!,
        // Generate new Python line
        newText: `        self.${newName} = ${nodeData.componentType}()`
      }]
    });
    
    setEditingNodeId(null);
  };

  // Custom node component with inline editing
  const EditableNode = ({ data, id }: any) => {
    const isEditing = editingNodeId === id;
    
    if (isEditing) {
      return (
        <div className="editable-node">
          <input
            autoFocus
            value={editValue}
            onChange={(e) => setEditValue(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Enter') {
                handleNodeEdit(id, editValue);
              } else if (e.key === 'Escape') {
                setEditingNodeId(null);
              }
            }}
            onBlur={() => handleNodeEdit(id, editValue)}
          />
        </div>
      );
    }
    
    return (
      <div className="node-content">
        <div className="node-label">{data.label}</div>
        {data.line && (
          <div className="node-line">Line {data.line}</div>
        )}
      </div>
    );
  };

  return (
    <div className="app">
      <div className="toolbar">
        <h2>🤖 System Editor (WYSIWYG)</h2>
        <div className="instructions">
          <strong>Click:</strong> Jump to code | 
          <strong>Double-click:</strong> Edit name | 
          <strong>Drag:</strong> Reorder
        </div>
        {selectedNode && (
          <div className="selected-info">
            Selected: <strong>{selectedNode.data.label}</strong>
            {selectedNode.data.line && ` (Line ${selectedNode.data.line})`}
          </div>
        )}
      </div>

      <div className="reactflow-wrapper">
        <ReactFlow
          nodes={nodes}
          edges={edges}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          onConnect={onConnect}
          onNodeClick={onNodeClick}
          onNodeDoubleClick={onNodeDoubleClick}
          nodeTypes={{
            default: EditableNode,
            input: EditableNode,
            output: EditableNode
          }}
          fitView
        >
          <Background variant="dots" gap={12} size={1} />
          <Controls />
          <MiniMap />
        </ReactFlow>
      </div>
    </div>
  );
}

export default App;


