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
 *   This file contains definition of the ToBLE platform driver which should be
 *   implemented by the Java application layer.
 *
 */

#ifndef OPENTHREAD_ANDROID_TOBLE_DRIVER_HPP_
#define OPENTHREAD_ANDROID_TOBLE_DRIVER_HPP_

#include <stdint.h>

#include <openthread/platform/toble.h>

namespace ot {

namespace Toble {

// A dummy class to handle C++ <--> Java conversions.
struct TobleConnection {};

class TobleDriver {
public:
    virtual ~TobleDriver() = default;

    virtual void Init() = 0;

    virtual void Process() = 0;

    virtual otError ScanStart(uint16_t aInterval, uint16_t aWindow, bool aActive) = 0;
    virtual otError ScanStop() = 0;
    void OnAdvReceived(otTobleAdvType aAdvType, otTobleAdvPacket *aAdvPacket);

    virtual TobleConnection *CreateConnection(const otTobleAddress *aPeerAddress,
                                                otTobleConnectionConfig *aConfig) = 0;
    virtual void Disconnect(TobleConnection *aConn) = 0;
    void OnConnected(TobleConnection *aConn);
    void OnDisconnected(TobleConnection *aConn);
    void OnConnectionIsReady(TobleConnection *aConn);

    virtual uint16_t GetMtu(TobleConnection *aConn) = 0;

    virtual otError C1Write(TobleConnection *aConn, const uint8_t *aBuffer, uint16_t aLength) = 0;
    void OnC1WriteDone(TobleConnection *aConn);

    virtual void C2Subscribe(TobleConnection *aConn, bool aSubscribe) = 0;
    void OnC2Subscribed(TobleConnection *aConn, bool aIsSubscribed);

    void OnC2Notification(TobleConnection *aConn,
                          const uint8_t *    aBuffer,
                          uint16_t           aLength);

    virtual otError AdvStart(const otTobleAdvConfig *aConfig) = 0;

    virtual otError AdvStop() = 0;

    virtual otError C2Notificate(TobleConnection *aConn, const uint8_t *aBuffer, uint16_t aLength) = 0;
};

} // namespace Toble

} // namespace ot

#endif // OPENTHREAD_ANDROID_TOBLE_DRIVER_HPP_
