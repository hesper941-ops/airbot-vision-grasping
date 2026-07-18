#!/usr/bin/env bash
set -e

echo "Prepare can1..."

sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

echo "Current can1 status:"
ip -details link show can1

echo "Start airbot_server on can1..."
sudo airbot_server -i can1 -p 50001
