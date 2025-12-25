#!/bin/bash
# Simple HTTP server to host the state visualizer

echo "Starting Orchestrator State Machine Visualizer..."
echo "Open your browser to: http://localhost:8000/state_visualizer.html"
echo ""
echo "Press Ctrl+C to stop"
echo ""

cd "$(dirname "$0")"
python3 -m http.server 8000
