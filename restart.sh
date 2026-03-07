#!/bin/bash
# Restart Brain main.py
# Usage: ./restart.sh

echo "Stopping main.py and all child processes..."
pkill -f "python3.*main.py" 2>/dev/null

sleep 2

# Kill any leftover python3 processes on port 5005
PIDS=$(ss -tlnp 2>/dev/null | grep 5005 | grep -oP 'pid=\K[0-9]+')
for PID in $PIDS; do
    kill "$PID" 2>/dev/null
done

sleep 1

# Verify port is free
if ss -tlnp 2>/dev/null | grep -q 5005; then
    echo "Port 5005 still in use, force killing..."
    PIDS=$(ss -tlnp 2>/dev/null | grep 5005 | grep -oP 'pid=\K[0-9]+')
    for PID in $PIDS; do
        kill -9 "$PID" 2>/dev/null
    done
    sleep 1
fi

echo "Starting main.py..."
cd /home/pi/Documents/Brain
nohup python3 -u main.py > /tmp/brain.log 2>&1 &
echo "PID=$!"

sleep 10

if ss -tlnp 2>/dev/null | grep -q 5005; then
    echo "OK - server running on port 5005"
else
    echo "FAIL - server not listening on port 5005"
    echo "Check /tmp/brain.log for errors"
fi
