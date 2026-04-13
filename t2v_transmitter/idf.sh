#!/bin/bash
# Transmitter: /dev/ttyACM1
source "$(dirname "$0")/../esp-idf/export.sh" > /dev/null 2>&1
ESPPORT=/dev/ttyACM1 idf.py "$@"
