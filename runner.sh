#!/bin/bash

cd tlv-processor
if pgrep -x "sensor" > /dev/null; then
    echo "Already running sensor!"
else
    echo "Starting sensor program!"
    ./sensor > sensor_log &
fi

if pgrep -x "radar" > /dev/null; then
    echo "Already running radar!"
else
    echo "Starting radar program!"
    ./radar > radar_log & 
fi

cd ..
cd scope-deepstream

if pgrep -x "smartscope" > /dev/null; then
    echo "Already running smartscope!"
else
    echo "Starting smartscope program!"
    ./smartscope $1
fi
