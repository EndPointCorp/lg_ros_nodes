#!/bin/bash

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo docker build -t lgros ${DOCKER_DIR}
