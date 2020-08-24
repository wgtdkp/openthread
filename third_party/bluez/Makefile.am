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

# Disable -Wunused-parameter for bluez library
CFLAGS                    += -Wno-unused-parameter
CXXFLAGS                  += -Wno-unused-parameter

# Disable -Wunused-function for bluez library
CFLAGS                    += -Wno-unused-function
CXXFLAGS                  += -Wno-unused-function

# Disable -Wshadow for bluez library
CFLAGS                    += -Wno-shadow
CXXFLAGS                  += -Wno-shadow

# Disable -Wpointer-arith for bluez library
CFLAGS                    += -Wno-pointer-arith
CXXFLAGS                  += -Wno-pointer-arith

# Disable -Wformat for bluez library
CFLAGS                    += -Wno-format
CXXFLAGS                  += -Wno-format

# Disable -Wpedantic for bluez library
CFLAGS                    += -Wno-pedantic
CXXFLAGS                  += -Wno-pedantic

# Disable -Wmissing-field-initializers for bluez library
CFLAGS                    += -Wno-missing-field-initializers
CXXFLAGS                  += -Wno-missing-field-initializers

# Disable -Wimplicit-function-declaration for bluez library
CFLAGS                    += -Wno-implicit-function-declaration
CXXFLAGS                  += -Wno-implicit-function-declaration

lib_LIBRARIES = libbluetooth-internal.a

lib_sources                                 = \
    repo/lib/bluetooth.c                      \
    repo/lib/hci.c                            \
    repo/lib/sdp.c                            \
    $(NULL)

lib_headers                                 = \
    repo/lib/bluetooth.h                      \
    repo/lib/hci.h                            \
    repo/lib/hci_lib.h                        \
   	repo/lib/sco.h                            \
    repo/lib/l2cap.h                          \
    repo/lib/sdp.h                            \
    repo/lib/sdp_lib.h                        \
    repo/lib/rfcomm.h                         \
    repo/lib/bnep.h                           \
    repo/lib/cmtp.h                           \
    repo/lib/hidp.h                           \
    $(NULL)

extra_headers                               = \
	repo/lib/mgmt.h                           \
	repo/lib/uuid.h                           \
	repo/lib/a2mp.h                           \
	repo/lib/amp.h                            \
    $(NULL)

extra_sources = repo/lib/uuid.c

libbluetooth_internal_a_CPPFLAGS            = \
    -I$(top_srcdir)/third_party/bluez         \
    -I$(top_srcdir)/third_party/bluez/repo    \
    $(GLIB_CFLAGS)                            \
    $(NULL)

libbluetooth_internal_a_SOURCES             = \
    $(lib_headers)                            \
    $(lib_sources)                            \
	$(extra_headers)                          \
    $(extra_sources)                          \
    $(NULL)

lib_LIBRARIES                              += libshared_mainloop.a

shared_sources                              = \
    repo/src/shared/io.h                      \
    repo/src/shared/timeout.h                 \
	repo/src/shared/queue.h                   \
    repo/src/shared/queue.c                   \
	repo/src/shared/util.h                    \
    repo/src/shared/util.c                    \
	repo/src/shared/mgmt.h                    \
    repo/src/shared/mgmt.c                    \
	repo/src/shared/crypto.h                  \
    repo/src/shared/crypto.c                  \
	repo/src/shared/ecc.h                     \
    repo/src/shared/ecc.c                     \
	repo/src/shared/ringbuf.h                 \
    repo/src/shared/ringbuf.c                 \
	repo/src/shared/tester.h                  \
    repo/src/shared/tester.c                  \
	repo/src/shared/hci.h                     \
    repo/src/shared/hci.c                     \
	repo/src/shared/hci-crypto.h              \
    repo/src/shared/hci-crypto.c              \
	repo/src/shared/hfp.h                     \
    repo/src/shared/hfp.c                     \
	repo/src/shared/uhid.h                    \
    repo/src/shared/uhid.c                    \
	repo/src/shared/pcap.h                    \
    repo/src/shared/pcap.c                    \
	repo/src/shared/btsnoop.h                 \
    repo/src/shared/btsnoop.c                 \
	repo/src/shared/ad.h                      \
    repo/src/shared/ad.c                      \
	repo/src/shared/att-types.h               \
	repo/src/shared/att.h                     \
    repo/src/shared/att.c                     \
	repo/src/shared/gatt-helpers.h            \
    repo/src/shared/gatt-helpers.c            \
	repo/src/shared/gatt-client.h             \
    repo/src/shared/gatt-client.c             \
	repo/src/shared/gatt-server.h             \
    repo/src/shared/gatt-server.c             \
	repo/src/shared/gatt-db.h                 \
    repo/src/shared/gatt-db.c                 \
	repo/src/shared/gap.h                     \
    repo/src/shared/gap.c                     \
	repo/src/shared/log.h                     \
    repo/src/shared/log.c                     \
	repo/src/shared/tty.h                     \
    $(NULL)

libshared_mainloop_a_CPPFLAGS               = \
    -D_DEFAULT_SOURCE                         \
    -I$(top_srcdir)/third_party/bluez         \
    -I$(top_srcdir)/third_party/bluez/repo    \
    $(GLIB_CFLAGS)                            \
    $(NULL)

libshared_mainloop_a_SOURCES                = \
    $(shared_sources)                         \
	repo/src/shared/io-mainloop.c             \
	repo/src/shared/timeout-mainloop.c        \
	repo/src/shared/mainloop.h                \
    repo/src/shared/mainloop.c                \
	repo/src/shared/mainloop-notify.h         \
	repo/src/shared/mainloop-notify.c         \
    $(NULL)

include $(abs_top_nlbuild_autotools_dir)/automake/post.am
