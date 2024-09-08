#!/bin/bash

CONTAINER_NAME="ros2_humble_workshop_container"
echo "Using Container Name: $CONTAINER_NAME"
docker exec -it "$CONTAINER_NAME" bash
