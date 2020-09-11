#!/bin/bash

readonly CUR_DIR=$(dirname "$(realpath -s $0)")
readonly PROJECT_DIR=${CUR_DIR}/..
readonly BUILD_DIR=${PROJECT_DIR}/build-toble

set -e

case "${TARGET}" in
    armv7a-linux-androideabi)
        ABI="armeabi-v7a"
        ;;
    aarch64-linux-android)
        ABI="arm64-v8a"
        ;;
    i686-linux-android)
        ABI="x86"
        ;;
    x86_64-linux-android)
        ABI="x86-64"
        ;;
    *)
        echo "invalid TARGET value: ${TARGET}"
        exit 1
esac

cd "${PROJECT_DIR}"

[ ! -d "${BUILD_DIR}" ] && ./bootstrap

mkdir -p "${BUILD_DIR}" && cd "${BUILD_DIR}"
cmake -GNinja \
     -DOT_FULL_LOGS=ON \
     -DOT_TOBLE=ON \
     -DOT_PLATFORM=android \
     -DCMAKE_TOOLCHAIN_FILE="${ANDROID_NDK_HOME}"/build/cmake/android.toolchain.cmake \
     -DANDROID_ABI="${ABI}" \
     -DANDROID_ARM_NEON=ON \
     -DANDROID_NATIVE_API_LEVEL="${API}" \
     ..
ninja toble-java

rm -rf libs && mkdir -p libs

## Create JAR library
javac -source 8 -target 8 examples/platforms/android/jni/io/openthread/toble/*.java

cd examples/platforms/android/jni
find ./io/openthread/toble -name "*.class" | xargs jar cvf "${BUILD_DIR}"/libs/libtoble.jar

cd "${BUILD_DIR}"

## Copy shared native libraries
for lib in $(find ./ -name "*.so"); do
    ## Avoid copying symblink files.
    [ ! -L "$lib" ] && cp "$lib" libs/
done
