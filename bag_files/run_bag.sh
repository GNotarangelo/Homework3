#!/bin/bash

OUTPUT_DIR="data"

if [ -d "$OUTPUT_DIR" ]; then
    echo "Folder '$OUTPUT_DIR' already exists. Overwriting..."
    rm -rf "$OUTPUT_DIR"
fi

# ROS 2 Humble command using the YAML override
ros2 bag record -o "$OUTPUT_DIR" \
    --qos-profile-overrides-path qos_override.yaml \
    /fmu/out/vehicle_local_position \
    /fmu/out/manual_control_setpoint \
    /fmu/out/actuator_outputs \
    /fmu/out/vehicle_attitude

    