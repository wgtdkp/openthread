#!/bin/bash

mkdir -p cmake-build
cd cmake-build

cmake -DOT_PLATFORM=posix \
      -DOT_DAEMON=ON \
      -DOT_BORDER_ROUTER=ON \
      -DOT_ECDSA=ON \
      -DOT_BACKBONE_ROUTER=ON \
      -DOT_THREAD_VERSION=1.2 \
      -DOT_SERVICE=ON \
      -DOT_FULL_LOGS=ON \
      -DOT_LOG_LEVEL=INFO \
      ..
