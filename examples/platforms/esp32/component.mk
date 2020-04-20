CFLAGS                                                                      += \
    -D_GNU_SOURCE                                                              \
    -DOPENTHREAD_CONFIG_FILE=\<openthread-config-esp32.h\>                     \
    -DOPENTHREAD_FTD=1                                                         \
    -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=\"openthread-core-esp32-config.h\"   \
    -DSPINEL_PLATFORM_HEADER=\"spinel_platform.h\"                             \
    -Wno-error=non-virtual-dtor

CXXFLAGS                                                                    += \
    -D_GNU_SOURCE                                                              \
    -DOPENTHREAD_CONFIG_FILE=\<openthread-config-esp32.h\>                     \
    -DOPENTHREAD_FTD=1                                                         \
    -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=\"openthread-core-esp32-config.h\"   \
    -DSPINEL_PLATFORM_HEADER=\"spinel_platform.h\"                             \
    -Wno-error=non-virtual-dtor

CPPFLAGS                                                                    += \
    -D_GNU_SOURCE                                                              \
    -DOPENTHREAD_CONFIG_FILE=\<openthread-config-esp32.h\>                     \
    -DOPENTHREAD_FTD=1                                                         \
    -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=\"openthread-core-esp32-config.h\"   \
    -DSPINEL_PLATFORM_HEADER=\"spinel_platform.h\"                             \
    -Wno-error=non-virtual-dtor

COMPONENT_DEPENDS := openthread
