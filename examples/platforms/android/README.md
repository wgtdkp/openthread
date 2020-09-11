# OpenThread(ToBLE) on Android

This directory includes platform drivers on Android, especially the BLE radio driver for ToBLE.

The `jni` sub-directory includes JNI wrapper of the `openthread/include/toble.h`  APIs that needs to be implemented on Android (in Java).

## Build

Bootstrap for the first build, and then:

```shell
## Set those env  variables to your proper values.
export ANDROID_NDK_HOME=Users/wgtdkp/Library/Android/sdk/ndk/21.3.6528147
## This is for pixel 3.
export ABI=armeabi-v7a
export API=21

mkdir -p build && cd build
cmake -GNinja \
     -DOT_TOBLE=ON \
     -DOT_PLATFORM=android \
     -DCMAKE_TOOLCHAIN_FILE="${ANDROID_NDK_HOME}"/build/cmake/android.toolchain.cmake \
     -DANDROID_ABI="${ABI}" \
     -DANDROID_ARM_NEON=ON \
     -DANDROID_NATIVE_API_LEVEL="${API}" \
     ..
ninja toble-java
```

You can find out generated Java files in `jni/io/openthread/toble`.
