import React, { useCallback, useState } from 'react';
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

// Define nodes with file paths
interface NodeData {
  label: string;
  filePath?: string;
  line?: number;
}

const initialNodes: Node<NodeData>[] = [
  { 
    id: '1', 
    type: 'input', 
    data: { 
      label: '🤖 System', 
      filePath: '/home/bam/bam_ws/src/like_keras/lk/common/system.py',
      line: 1 
    }, 
    position: { x: 250, y: 25 } 
  },
  { 
    id: '2', 
    data: { 
      label: '🤖 Robot Description', 
      filePath: '/home/bam/bam_ws/src/like_keras/lk/common/robot_description/robot_description.py',
      line: 1 
    }, 
    position: { x: 100, y: 150 } 
  },
  { 
    id: '3', 
    data: { 
      label: '📡 Diagnostics', 
      filePath: '/home/bam/bam_ws/src/like_keras/lk/msgs/diagnostics.py',
      line: 1 
    }, 
    position: { x: 400, y: 150 } 
  },
  { 
    id: '4', 
    type: 'output', 
    data: { 
      label: '🎬 Agent', 
      filePath: '/home/bam/bam_ws/src/like_keras/lk/agent/agent.py',
      line: 1 
    }, 
    position: { x: 250, y: 275 } 
  },
];

const initialEdges: Edge[] = [
  { id: 'e1-2', source: '1', target: '2', animated: true },
  { id: 'e1-3', source: '1', target: '3', animated: true },
  { id: 'e2-4', source: '2', target: '4' },
  { id: 'e3-4', source: '3', target: '4' },
];

interface FileButtonsProps {
  path: string;
  line: number;
  title: string;
}

const FileButtons: React.FC<FileButtonsProps> = ({ path, line, title }) => {
  const openFile = (viewColumn: number) => {
    vscode.postMessage({ type: 'openFile', path, line, viewColumn });
  };

  return (
    <div className="file-item">
      <div className="file-title">{title}</div>
      <div className="buttons">
        <button onClick={() => openFile(1)}>← Left</button>
        <button onClick={() => openFile(2)}>Right →</button>
        <button onClick={() => openFile(-1)}>📍 Here</button>
      </div>
    </div>
  );
};

function App() {
  const [nodes, setNodes, onNodesChange] = useNodesState(initialNodes);
  const [edges, setEdges, onEdgesChange] = useEdgesState(initialEdges);
  const [timing, setTiming] = useState<string>('');
  const [lastClickedNode, setLastClickedNode] = useState<string>('');

  const onConnect = useCallback(
    (params: Connection) => setEdges((eds) => addEdge(params, eds)),
    [setEdges]
  );

  // Handle node click to open file
  const onNodeClick: NodeMouseHandler = useCallback((event, node) => {
    const nodeData = node.data as NodeData;
    if (nodeData.filePath) {
      setLastClickedNode(node.id);
      vscode.postMessage({ 
        type: 'openFile', 
        path: nodeData.filePath, 
        line: nodeData.line || 1, 
        viewColumn: 2  // Open in right panel by default
      });
    }
  }, []);

  // Listen for messages from extension
  React.useEffect(() => {
    const handleMessage = (event: MessageEvent) => {
      const msg = event.data;
      if (msg.type === 'fileOpened') {
        const col = msg.viewColumn === 1 ? 'Left' : msg.viewColumn === 2 ? 'Right' : 'Active';
        setTiming(`✅ Opened in ${col} panel | ${msg.elapsed}ms`);
        setTimeout(() => setTiming(''), 3000);
      }
    };

    window.addEventListener('message', handleMessage);
    return () => window.removeEventListener('message', handleMessage);
  }, []);

  return (
    <div className="app">
      <div className="floating-panel">
        <h2>⚡ LK File Navigator</h2>
        {timing && <div className="timing">{timing}</div>}
        {lastClickedNode && (
          <div className="info-box">
            💡 Click nodes in the graph to open files!<br/>
            Last clicked: Node {lastClickedNode}
          </div>
        )}
        
        <h3>Quick Links</h3>
        <FileButtons
          path="/home/bam/bam_ws/src/like_keras/lk_viewer/features.md"
          line={20}
          title="📄 features.md:20"
        />
        
        <FileButtons
          path="/home/bam/bam_ws/src/like_keras/lk_viewer/README.md"
          line={79}
          title="📄 README.md:79"
        />
        
        <FileButtons
          path="/home/bam/bam_ws/src/like_keras/lk/msgs/diagnostics.py"
          line={30}
          title="🐍 diagnostics.py:30"
        />
      </div>

      <div className="reactflow-wrapper">
        <ReactFlow
          nodes={nodes}
          edges={edges}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          onConnect={onConnect}
          onNodeClick={onNodeClick}
          fitView
        >
          <Background variant="dots" gap={12} size={1} />
          <Controls />
          <MiniMap nodeColor="#555" />
        </ReactFlow>
      </div>
    </div>
  );
}

export default App;

