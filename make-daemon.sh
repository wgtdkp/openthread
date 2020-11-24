#!/bin/bash

mkdir -p cmake-build-daemon
cd cmake-build-daemon

cmake -DOT_PLATFORM=posix \
      -DOT_DAEMON=ON \
      -DOT_UDP_FORWARD=OFF \
      -DOT_DUCKHORN_BORDER_ROUTER=ON \
      -DOT_BORDER_AGENT=ON \
      -DOT_BORDER_ROUTER=ON \
      -DOT_ECDSA=ON \
      -DOT_LOG_OUTPUT=PLATFORM_DEFINED \
      -DOT_SERVICE=ON \
      -DOT_THREAD_VERSION=1.1 \
      -DOT_FULL_LOGS=OFF \
      -DOT_LOG_LEVEL=DEBG \
      -DOT_READLINE=OFF \
      -GNinja ..

ninja
