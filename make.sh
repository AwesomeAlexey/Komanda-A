#!/bin/bash

set -e
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
cd ..
