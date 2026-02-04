#!/bin/bash

echo "=== Network Check ==="
echo ""
echo "Local IP:"
ip addr show enp88s0 | grep "inet " | awk '{print $2}' | cut -d/ -f1
echo ""
echo "ROS 2 nodes:"
ros2 node list 2>/dev/null
echo ""
echo "ROS 2 topics:"
ros2 topic list 2>/dev/null
echo ""
echo "micro-ROS agent processes:"
ps aux | grep -E "(micro-ros|agent)" | grep -v grep
echo ""
echo "Listening on port 8888:"
netstat -tulnp 2>/dev/null | grep 8888 || echo "Port 8888 not found"
echo ""
