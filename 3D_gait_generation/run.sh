#!/bin/sh
# runs the step up program

cd ./src
LD_PRELOAD=/usr/local/lib/libomp.so python3 ./step-up.py
cd ..