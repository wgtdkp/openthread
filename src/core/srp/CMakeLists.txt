add_library(ot-srp-client
    srp_client.cpp
    srp-thread.c
)

target_link_libraries(ot-srp-client
    PRIVATE
        srp-client
        ot-config
        ${OT_MBEDTLS}
)
