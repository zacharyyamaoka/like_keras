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

const PYTHON_BACKEND_URL = 'http://localhost:8765';

interface NodeData {
  label: string;
  filePath?: string;
  line?: number;
  classes?: string[];
  functions?: string[];
}

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
  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);
  const [timing, setTiming] = useState<string>('');
  const [lastClickedNode, setLastClickedNode] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string>('');

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
        viewColumn: 2
      });
    }
  }, []);

  // Fetch graph data from Python backend
  useEffect(() => {
    const fetchGraphData = async () => {
      try {
        setLoading(true);
        setError('');
        
        const response = await fetch(`${PYTHON_BACKEND_URL}/parse?directory=lk`);
        
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        
        if (data.error) {
          throw new Error(data.error);
        }
        
        console.log('Received graph data:', data);
        
        // Set nodes and edges from Python backend
        setNodes(data.graph.nodes);
        setEdges(data.graph.edges);
        
        setLoading(false);
      } catch (err: any) {
        console.error('Error fetching graph data:', err);
        setError(err.message || 'Failed to connect to Python backend');
        setLoading(false);
        
        // Fallback to static nodes if backend unavailable
        setNodes([
          { 
            id: '1', 
            type: 'input', 
            data: { 
              label: '⚠️ Backend Offline', 
              filePath: '/home/bam/bam_ws/src/like_keras/lk/common/system.py',
              line: 1 
            }, 
            position: { x: 250, y: 25 } 
          },
        ]);
      }
    };

    // Delay to give backend time to start
    const timer = setTimeout(fetchGraphData, 1000);
    return () => clearTimeout(timer);
  }, [setNodes, setEdges]);

  // Listen for messages from extension
  useEffect(() => {
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

  const refreshGraph = async () => {
    setLoading(true);
    setError('');
    
    try {
      const response = await fetch(`${PYTHON_BACKEND_URL}/parse?directory=lk`);
      const data = await response.json();
      
      setNodes(data.graph.nodes);
      setEdges(data.graph.edges);
      setTiming('✅ Graph refreshed!');
      setTimeout(() => setTiming(''), 2000);
    } catch (err: any) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="app">
      <div className="floating-panel">
        <h2>⚡ LK Viewer</h2>
        
        {loading && <div className="info-box">🔄 Loading graph from Python backend...</div>}
        {error && <div className="error-box">❌ {error}</div>}
        {timing && <div className="timing">{timing}</div>}
        
        {!loading && !error && (
          <div className="info-box">
            💡 Click nodes to open files!<br/>
            {lastClickedNode && `Last: Node ${lastClickedNode}`}<br/>
            <button className="refresh-btn" onClick={refreshGraph}>🔄 Refresh Graph</button>
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

