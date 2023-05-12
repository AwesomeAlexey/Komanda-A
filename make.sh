#!/bin/bash

set -e
mkdir -p build
mkdir -p dump
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
cd ..
