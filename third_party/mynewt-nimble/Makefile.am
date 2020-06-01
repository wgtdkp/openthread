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

include $(abs_top_nlbuild_autotools_dir)/automake/pre.am

NIMBLE_ROOT               := $(top_srcdir)/third_party/mynewt-nimble/repo

EXTRA_DIST                 = $(NULL)

lib_LIBRARIES              = libnimble.a

# Do not enable -pedantic-errors for mynewt-nimble
override CFLAGS           := $(filter-out -pedantic-errors,$(CFLAGS))
override CXXFLAGS         := $(filter-out -pedantic-errors,$(CXXFLAGS))

# Do not enable -Werror for mynewt-nimble
CFLAGS                    += -Wno-error
CXXFLAGS                  += -Wno-error

# Do not enable -Wunused-parameter for mynewt-nimble
CFLAGS                    += -Wno-unused-parameter
CXXFLAGS                  += -Wno-unused-parameter

# Do not enable -Wundef for mynewt-nimble
CFLAGS                    += -Wno-undef
CXXFLAGS                  += -Wno-undef

# Do not enable -Wsign-compare for mynewt-nimble
CFLAGS                    += -Wno-sign-compare
CXXFLAGS                  += -Wno-sign-compare

# Do not enable -Wold-style-declaration for mynewt-nimble
CFLAGS                    += -Wno-old-style-declaration

# Do not enable -Wtype-limits for mynewt-nimble
CFLAGS                    += -Wno-type-limits
CXXFLAGS                  += -Wno-type-limits

# Do not enable strict C99 checks
override CFLAGS           := $(filter-out -std=c99,$(CFLAGS))
override CXXFLAGS         := $(filter-out -std=c99,$(CXXFLAGS))

CFLAGS += -std=c11

NIMBLE_SRC                                              = \
    repo/nimble/host/src/ble_att.c                        \
    repo/nimble/host/src/ble_att_clt.c                    \
    repo/nimble/host/src/ble_att_cmd.c                    \
    repo/nimble/host/src/ble_att_svr.c                    \
    repo/nimble/host/src/ble_eddystone.c                  \
    repo/nimble/host/src/ble_gap.c                        \
    repo/nimble/host/src/ble_gattc.c                      \
    repo/nimble/host/src/ble_gatts.c                      \
    repo/nimble/host/src/ble_hs_adv.c                     \
    repo/nimble/host/src/ble_hs_atomic.c                  \
    repo/nimble/host/src/ble_hs.c                         \
    repo/nimble/host/src/ble_hs_cfg.c                     \
    repo/nimble/host/src/ble_hs_conn.c                    \
    repo/nimble/host/src/ble_hs_dbg.c                     \
    repo/nimble/host/src/ble_hs_flow.c                    \
    repo/nimble/host/src/ble_hs_hci.c                     \
    repo/nimble/host/src/ble_hs_hci_cmd.c                 \
    repo/nimble/host/src/ble_hs_hci_evt.c                 \
    repo/nimble/host/src/ble_hs_hci_util.c                \
    repo/nimble/host/src/ble_hs_id.c                      \
    repo/nimble/host/src/ble_hs_log.c                     \
    repo/nimble/host/src/ble_hs_mbuf.c                    \
    repo/nimble/host/src/ble_hs_mqueue.c                  \
    repo/nimble/host/src/ble_hs_misc.c                    \
    repo/nimble/host/src/ble_hs_periodic_sync.c           \
    repo/nimble/host/src/ble_hs_pvcy.c                    \
    repo/nimble/host/src/ble_hs_startup.c                 \
    repo/nimble/host/src/ble_hs_stop.c                    \
    repo/nimble/host/src/ble_ibeacon.c                    \
    repo/nimble/host/src/ble_l2cap.c                      \
    repo/nimble/host/src/ble_l2cap_coc.c                  \
    repo/nimble/host/src/ble_l2cap_sig.c                  \
    repo/nimble/host/src/ble_l2cap_sig_cmd.c              \
    repo/nimble/host/src/ble_monitor.c                    \
    repo/nimble/host/src/ble_sm_alg.c                     \
    repo/nimble/host/src/ble_sm.c                         \
    repo/nimble/host/src/ble_sm_cmd.c                     \
    repo/nimble/host/src/ble_sm_lgcy.c                    \
    repo/nimble/host/src/ble_sm_sc.c                      \
    repo/nimble/host/src/ble_store.c                      \
    repo/nimble/host/src/ble_store_util.c                 \
    repo/nimble/host/src/ble_uuid.c                       \
    repo/nimble/host/services/ans/src/ble_svc_ans.c       \
    repo/nimble/host/services/bas/src/ble_svc_bas.c       \
    repo/nimble/host/services/gap/src/ble_svc_gap.c       \
    repo/nimble/host/services/gatt/src/ble_svc_gatt.c     \
    repo/nimble/host/services/ias/src/ble_svc_ias.c       \
    repo/nimble/host/services/lls/src/ble_svc_lls.c       \
    repo/nimble/host/services/tps/src/ble_svc_tps.c       \
    repo/nimble/host/store/ram/src/ble_store_ram.c        \
    repo/nimble/host/util/src/addr.c                      \
    repo/nimble/src/ble_util.c                            \
    $(NULL)

# Few utils and data structures copied from Nimble
NIMBLE_SRC                                             += \
    repo/porting/nimble/src/nimble_port.c                 \
    $(NULL)

# Few utils and data structures copied from Nimble
NIMBLE_SRC                                             += \
    repo/porting/nimble/src/endian.c                      \
    repo/porting/nimble/src/mem.c                         \
    repo/porting/nimble/src/os_mbuf.c                     \
    repo/porting/nimble/src/os_mempool.c                  \
    repo/porting/nimble/src/os_msys_init.c                \
    $(NULL)

NIMBLE_SRC                                             += \
    repo/ext/tinycrypt/src/aes_decrypt.c                  \
    repo/ext/tinycrypt/src/aes_encrypt.c                  \
    repo/ext/tinycrypt/src/cmac_mode.c                    \
    repo/ext/tinycrypt/src/ecc.c                          \
    repo/ext/tinycrypt/src/ecc_dh.c                       \
    repo/ext/tinycrypt/src/utils.c                        \
    $(NULL)

libnimble_a_SOURCES                                     = \
    $(NIMBLE_SRC)                                         \
    $(NULL)

libnimble_a_CPPFLAGS                                    = \
    -I$(NIMBLE_ROOT)/nimble/include                       \
    -I$(NIMBLE_ROOT)/nimble/host/include                  \
    -I$(NIMBLE_ROOT)/nimble/host/services/ans/include     \
    -I$(NIMBLE_ROOT)/nimble/host/services/bas/include     \
    -I$(NIMBLE_ROOT)/nimble/host/services/bleuart/include \
    -I$(NIMBLE_ROOT)/nimble/host/services/gap/include     \
    -I$(NIMBLE_ROOT)/nimble/host/services/gatt/include    \
    -I$(NIMBLE_ROOT)/nimble/host/services/ias/include     \
    -I$(NIMBLE_ROOT)/nimble/host/services/lls/include     \
    -I$(NIMBLE_ROOT)/nimble/host/services/tps/include     \
    -I$(NIMBLE_ROOT)/nimble/host/store/ram/include        \
    -I$(NIMBLE_ROOT)/nimble/host/util/include             \
    -I$(NIMBLE_ROOT)/ext/tinycrypt/include                \
    $(NULL)

# Source files for NPL OSAL
if OPENTHREAD_PLATFORM_POSIX
libnimble_a_SOURCES                                    += \
    repo/porting/npl/linux/src/os_atomic.c                \
    repo/porting/npl/linux/src/os_callout.c               \
    repo/porting/npl/linux/src/os_eventq.cc               \
    repo/porting/npl/linux/src/os_mutex.c                 \
    repo/porting/npl/linux/src/os_sem.c                   \
    repo/porting/npl/linux/src/os_task.c                  \
    repo/porting/npl/linux/src/os_time.c                  \
    repo/nimble/transport/socket/src/ble_hci_socket.c     \
    $(NULL)

libnimble_a_CPPFLAGS                                   += \
    -D_GNU_SOURCE                                         \
    -DMYNEWT_VAL_BLE_SOCK_USE_LINUX_BLUE=1                \
    -DMYNEWT_VAL_BLE_SOCK_LINUX_DEV=0                     \
    -I$(top_srcdir)/include                               \
    -I$(NIMBLE_ROOT)/../include                           \
    -I$(NIMBLE_ROOT)/porting/nimble/include               \
    -I$(NIMBLE_ROOT)/porting/npl/linux/include            \
    -I$(NIMBLE_ROOT)/porting/npl/linux/src                \
    -I$(NIMBLE_ROOT)/nimble/transport/socket/include      \
    $(NULL)
endif # OPENTHREAD_PLATFORM_POSIX

if OPENTHREAD_BUILD_COVERAGE
Dash                                          = -
CLEANFILES                                    = $(shell find $(top_builddir)/third_party/mynewt-nimble $(Dash)name "*.gcda" $(Dash)o $(Dash)name "*.gcno")
endif # OPENTHREAD_BUILD_COVERAGE

include $(abs_top_nlbuild_autotools_dir)/automake/post.am
