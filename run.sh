#!/bin/bash

set -e
sudo chrt -r 50 ./build/control_system -c configs/config.json -f configs/sliding_controller.json 
