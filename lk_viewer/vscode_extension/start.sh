#!/bin/bash

# Simple script to test the LK Viewer Extension

echo "🚀 Starting LK Viewer Extension..."
echo ""
echo "Step 1: Opening extension folder in Cursor..."
cursor /home/bam/bam_ws/src/like_keras/lk_viewer/vscode_extension

echo ""
echo "Step 2: Instructions"
echo "===================="
echo ""
echo "Once Cursor opens:"
echo "1. Press F5 (or go to Run > Start Debugging)"
echo "2. A new 'Extension Development Host' window will open"
echo "3. In the new window, you'll see:"
echo "   - LK Viewer icon in the sidebar (left)"
echo "   - Or use Command Palette: 'LK Viewer: Open File Panel'"
echo ""
echo "To test:"
echo "- Click on file links in the sidebar panel"
echo "- Files will open INSTANTLY (< 10ms)!"
echo ""
echo "To use Custom Graph Editor:"
echo "- Right-click on a *_config.py file"
echo "- Select 'Open With...' > 'LK Graph Editor'"
echo ""


