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

/**
 * @file
 *   This file contains APIs to run the ToBLE stack on Android.
 */

#ifndef OPENTHREAD_ANDROID_TOBLE_HPP_
#define OPENTHREAD_ANDROID_TOBLE_HPP_

#include <string>

#include <stdint.h>

#include <openthread/error.h>
#include <openthread/instance.h>
#include <openthread/message.h>

#include "toble_driver.hpp"

namespace ot {

namespace Toble {

class TobleCallbacks {
public:
    TobleCallbacks() = default;
    virtual ~TobleCallbacks() = default;

    virtual void OnIp6Receive(const uint8_t *aPacket, size_t aPacketLength) = 0;
};

class Toble {
public:
    static Toble* GetInstance();

    /**
     * This method initializes ToBLE.
     *
     * @returns Link-local address of current device.
     *
     */
    std::string Init(TobleCallbacks *aTobleCallbacks, TobleDriver *aTobleDriver);

    /**
     * This method deinitializes ToBLE.
     *
     */
    void Deinit();

    /**
     * This method processes ToBLE event.
     *
     * @returns Maximum delay of next call of this function. In milliseconds.
     *
     */
    uint32_t Process();

    /**
     * This method sends IPv6 packets via ToBLE.
     *
     * @note The packet is send with link security disabled.
     *
     */
    otError Ip6Send(const uint8_t *aPacket, size_t aPacketLength);

    TobleDriver *GetTobleDriver();

    otInstance *GetOtInstance();

private:
    Toble() = default;

    static void Ip6ReceiveCallback(otMessage *aMessage, void *aContext);

    TobleCallbacks *mTobleCallbacks = nullptr;
    TobleDriver *mTobleDriver = nullptr;
    otInstance *mInstance = nullptr;
};

} // namespace Toble

} // namespace ot

#endif // OPENTHREAD_ANDROID_TOBLE_HPP_
