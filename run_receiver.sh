#!/bin/bash

source /opt/ros/humble/setup.bash
export LOG_LEVEL=0
./build/receiver_node config.json
