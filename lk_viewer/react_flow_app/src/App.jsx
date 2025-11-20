import { useCallback, useEffect, useState } from 'react'
import ReactFlow, {
  Background,
  Controls,
  MiniMap,
  addEdge,
  useNodesState,
  useEdgesState,
  MarkerType,
} from '@xyflow/react'
import '@xyflow/react/dist/style.css'

const initialNodes = []
const initialEdges = []

let nodeIdCounter = 0

function App() {
  const [nodes, setNodes, onNodesChange] = useNodesState(initialNodes)
  const [edges, setEdges, onEdgesChange] = useEdgesState(initialEdges)
  const [ws, setWs] = useState(null)
  const [isConnected, setIsConnected] = useState(false)
  const [isReceivingUpdate, setIsReceivingUpdate] = useState(false)

  // WebSocket connection
  useEffect(() => {
    const websocket = new WebSocket('ws://localhost:8000/ws/nodes')
    
    websocket.onopen = () => {
      console.log('WebSocket connected')
      setIsConnected(true)
      setWs(websocket)
      // Request initial state
      websocket.send(JSON.stringify({ type: 'get_nodes' }))
    }

    websocket.onmessage = (event) => {
      const data = JSON.parse(event.data)
      
      if (data.type === 'nodes_update') {
        // Update nodes from server
        setIsReceivingUpdate(true)
        setNodes(data.nodes || [])
        setTimeout(() => setIsReceivingUpdate(false), 100)
      } else if (data.type === 'init') {
        // Initial state from server
        if (data.nodes) {
          setIsReceivingUpdate(true)
          setNodes(data.nodes)
          setTimeout(() => setIsReceivingUpdate(false), 100)
          // Update nodeIdCounter to avoid conflicts
          if (data.nodes.length > 0) {
            const maxId = Math.max(...data.nodes.map(n => {
              const idNum = parseInt(n.id.replace('node-', ''))
              return isNaN(idNum) ? 0 : idNum
            }))
            nodeIdCounter = maxId + 1
          }
        }
      }
    }

    websocket.onerror = (error) => {
      console.error('WebSocket error:', error)
    }

    websocket.onclose = () => {
      console.log('WebSocket disconnected')
      setIsConnected(false)
      setWs(null)
    }

    return () => {
      if (websocket) {
        websocket.close()
      }
    }
  }, [setNodes])

  // Send node updates to server
  const sendNodeUpdate = useCallback((updatedNodes) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'nodes_update',
        nodes: updatedNodes,
      }))
    }
  }, [ws])

  // Send node updates when nodes change (debounced)
  // Skip if we're receiving an update from server to prevent feedback loop
  useEffect(() => {
    if (!isReceivingUpdate && nodes.length >= 0) {
      const timeoutId = setTimeout(() => {
        sendNodeUpdate(nodes)
      }, 300) // Debounce updates
      return () => clearTimeout(timeoutId)
    }
  }, [nodes, sendNodeUpdate, isReceivingUpdate])

  // Handle node changes (drag, position, etc.)
  const handleNodesChange = useCallback((changes) => {
    onNodesChange(changes)
  }, [onNodesChange])

  // Handle node drag stop - send update immediately
  const handleNodeDragStop = useCallback((event, node) => {
    sendNodeUpdate(nodes)
  }, [nodes, sendNodeUpdate])

  // Add new node on double-click
  const onPaneDoubleClick = useCallback((event) => {
    const newNode = {
      id: `node-${nodeIdCounter++}`,
      type: 'default',
      position: { x: event.clientX - 100, y: event.clientY - 100 },
      data: { label: `Node ${nodeIdCounter - 1}` },
    }
    
    const updatedNodes = [...nodes, newNode]
    setNodes(updatedNodes)
    sendNodeUpdate(updatedNodes)
  }, [nodes, setNodes, sendNodeUpdate])

  // Handle node label editing
  const onNodeDoubleClick = useCallback((event, node) => {
    const newLabel = prompt('Enter node name:', node.data.label || '')
    if (newLabel !== null) {
      const updatedNodes = nodes.map((n) =>
        n.id === node.id ? { ...n, data: { ...n.data, label: newLabel } } : n
      )
      setNodes(updatedNodes)
      sendNodeUpdate(updatedNodes)
    }
  }, [nodes, setNodes, sendNodeUpdate])

  // Add node button handler
  const handleAddNode = useCallback(() => {
    const newNode = {
      id: `node-${nodeIdCounter++}`,
      type: 'default',
      position: { x: Math.random() * 400, y: Math.random() * 400 },
      data: { label: `Node ${nodeIdCounter - 1}` },
    }
    
    const updatedNodes = [...nodes, newNode]
    setNodes(updatedNodes)
    sendNodeUpdate(updatedNodes)
  }, [nodes, setNodes, sendNodeUpdate])

  const onConnect = useCallback((params) => {
    setEdges((eds) => addEdge(params, eds))
  }, [setEdges])

  return (
    <div style={{ width: '100vw', height: '100vh' }}>
      <div style={{ position: 'absolute', top: 10, left: 10, zIndex: 10, padding: '10px', background: 'white', borderRadius: '5px', boxShadow: '0 2px 5px rgba(0,0,0,0.2)' }}>
        <button onClick={handleAddNode} style={{ padding: '8px 16px', marginRight: '10px', cursor: 'pointer' }}>
          Add Node
        </button>
        <span style={{ color: isConnected ? 'green' : 'red' }}>
          {isConnected ? '● Connected' : '● Disconnected'}
        </span>
      </div>
      <ReactFlow
        nodes={nodes}
        edges={edges}
        onNodesChange={handleNodesChange}
        onEdgesChange={onEdgesChange}
        onConnect={onConnect}
        onNodeDragStop={handleNodeDragStop}
        onPaneDoubleClick={onPaneDoubleClick}
        onNodeDoubleClick={onNodeDoubleClick}
        fitView
      >
        <Background />
        <Controls />
        <MiniMap />
      </ReactFlow>
    </div>
  )
}

export default App

