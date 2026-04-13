#!/bin/bash
# Receiver: /dev/ttyACM0
source "$(dirname "$0")/../esp-idf/export.sh" > /dev/null 2>&1
ESPPORT=/dev/ttyACM0 idf.py "$@"
