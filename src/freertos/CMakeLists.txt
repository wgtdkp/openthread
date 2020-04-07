#
#  Copyright (c) 2020, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

add_library(openthread-freertos)

target_sources(openthread-freertos
    PRIVATE ot_rtos_impl.cpp
)

target_include_directories(openthread-freertos
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## This is a test app.
#add_executable(openthread-freertos-esp32)
#
#target_sources(openthread-freertos-esp32
#    PRIVATE esp32_app_main.c
#)
#
#target_link_libraries(openthread-freertos-esp32
#    PRIVATE
#        idf::esp32
#        idf::freertos
#        idf::spi_flash
#        openthread-ftd
#        openthread-esp32
#        openthread-freertos
#)
#
#if (NOT DEFINED ENV{IDF_PATH})
#    set(ENV{IDF_PATH} ${PROJECT_SOURCE_DIR}/third_party/esp-idf/repo)
#endif()
#include($ENV{IDF_PATH}/tools/cmake/idf.cmake)
#
#idf_build_executable(openthread-freertos-esp32)
#