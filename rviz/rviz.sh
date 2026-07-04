#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
xhost +local:docker
docker compose -f docker-compose.rviz.yml up --build
