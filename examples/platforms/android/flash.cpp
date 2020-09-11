/*
 *  Copyright (c) 2020, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "openthread-android-config.h"
#include "platform-android.h"

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <openthread/config.h>
#include <openthread/platform/flash.h>

#include "core/common/logging.hpp"
#include "lib/platform/exit_code.h"

enum
{
    SWAP_SIZE = 2048,
    SWAP_NUM  = 2,
};

static uint8_t sMemoryFile[1024 * 16];

void otPlatFlashInit(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    memset(sMemoryFile, 0xff, sizeof(sMemoryFile));
}

uint32_t otPlatFlashGetSwapSize(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return SWAP_SIZE;
}

void otPlatFlashErase(otInstance *aInstance, uint8_t aSwapIndex)
{
    OT_UNUSED_VARIABLE(aInstance);

    uint8_t  buffer[SWAP_SIZE];
    uint32_t address;
    ssize_t  rval;

    assert(aSwapIndex < SWAP_NUM);

    address = aSwapIndex ? SWAP_SIZE : 0;
    memset(buffer, 0xff, sizeof(buffer));

    memcpy(sMemoryFile + address, buffer, sizeof(buffer));
}

void otPlatFlashRead(otInstance *aInstance, uint8_t aSwapIndex, uint32_t aOffset, void *aData, uint32_t aSize)
{
    OT_UNUSED_VARIABLE(aInstance);

    uint32_t address;
    ssize_t  rval;

    assert((aSwapIndex < SWAP_NUM) && (aSize <= SWAP_SIZE) && (aOffset <= (SWAP_SIZE - aSize)));

    address = aSwapIndex ? SWAP_SIZE : 0;

    memcpy(aData, sMemoryFile + address + aOffset, aSize);
}

void otPlatFlashWrite(otInstance *aInstance, uint8_t aSwapIndex, uint32_t aOffset, const void *aData, uint32_t aSize)
{
    OT_UNUSED_VARIABLE(aInstance);

    uint32_t address;
    uint8_t  byte;
    ssize_t  rval;

    assert((aSwapIndex < SWAP_NUM) && (aSize <= SWAP_SIZE) && (aOffset <= (SWAP_SIZE - aSize)));

    address = aSwapIndex ? SWAP_SIZE : 0;
    address += aOffset;

    for (uint32_t offset = 0; offset < aSize; offset++)
    {
        byte = sMemoryFile[address + offset];

        // Use bitwise AND to emulate the behavior of flash memory
        byte &= ((uint8_t *)aData)[offset];

        sMemoryFile[address + offset] = byte;
    }
}
