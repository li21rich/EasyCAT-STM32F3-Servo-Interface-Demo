#!/bin/bash
# Kill any leftover instance from a previous run
pkill -f pdserv-example-st 2>/dev/null
sleep 0.5

# Launch pdserv example in background, then Testmanager in foreground
pdserv-example-st &
PDSERV_PID=$!
sleep 2
testmanager-ng -c /usr/share/testmanager-ng/example.tml

# When you close the Testmanager GUI, kill the background pdserv process too
kill $PDSERV_PID 2>/dev/null
