#!/bin/bash

source /opt/ros/humble/setup.bash
export LOG_LEVEL=0
./build/sender_node config.json 
