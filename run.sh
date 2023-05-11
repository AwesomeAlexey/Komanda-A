#!/bin/bash

set -e
sudo ./build/control_system -c configs/config.json -f configs/sliding_controller.json 
