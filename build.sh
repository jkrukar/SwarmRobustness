#!/bin/bash
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DARGOS_BUILD_NATIVE=ON .. && make

