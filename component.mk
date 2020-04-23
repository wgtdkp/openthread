COMPONENT_ADD_INCLUDEDIRS                   := \
	examples/platforms/esp32/include           \
	examples/platforms/esp32                   \
	third_party/jlink/SEGGER_RTT_V640/RTT      \
	include                                    \
	src/core                                   \
	src/ncp                                    \
	src/lib/hdlc                               \
	src/lib/spinel

COMPONENT_PRIV_INCLUDEDIRS := \
	src

COMPONENT_SRCDIRS         := \
	src/cli                  \
	src/core                 \
	src/core/api             \
	src/core/coap            \
	src/core/common          \
	src/core/crypto          \
	src/core/mac             \
	src/core/meshcop         \
	src/core/net             \
	src/core/radio           \
	src/core/thread          \
	src/core/utils           \
	src/ncp                  \
	src/lib/hdlc             \
	src/lib/spinel           \
	examples/platforms/esp32 \

CFLAGS                                                                      += \
    -D_GNU_SOURCE                                                              \
    -DOPENTHREAD_CONFIG_FILE=\<openthread-core-esp32-config.h\>                \
    -DOPENTHREAD_FTD=1                                                         \
	-DOPENTHREAD_SPINEL_CONFIG_OPENTHREAD_MESSAGE_ENABLE=1                     \
    -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=\"openthread-core-esp32-config.h\"   \
    -DSPINEL_PLATFORM_HEADER=\"spinel_platform.h\"                             \
    -Wno-error=non-virtual-dtor

CXXFLAGS                                                                    += \
    -D_GNU_SOURCE                                                              \
    -DOPENTHREAD_CONFIG_FILE=\<openthread-core-esp32-config.h\>                \
    -DOPENTHREAD_FTD=1                                                         \
	-DOPENTHREAD_SPINEL_CONFIG_OPENTHREAD_MESSAGE_ENABLE=1                     \
    -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=\"openthread-core-esp32-config.h\"   \
    -DSPINEL_PLATFORM_HEADER=\"spinel_platform.h\"                             \
    -Wno-error=non-virtual-dtor

CPPFLAGS                                                                    += \
    -D_GNU_SOURCE                                                              \
    -DOPENTHREAD_CONFIG_FILE=\<openthread-core-esp32-config.h\>                \
    -DOPENTHREAD_FTD=1                                                         \
	-DOPENTHREAD_SPINEL_CONFIG_OPENTHREAD_MESSAGE_ENABLE=1                     \
    -DOPENTHREAD_PROJECT_CORE_CONFIG_FILE=\"openthread-core-esp32-config.h\"   \
    -DSPINEL_PLATFORM_HEADER=\"spinel_platform.h\"                             \
    -Wno-error=non-virtual-dtor
