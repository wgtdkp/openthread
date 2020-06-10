/*
 *  Copyright (c) 2019, The OpenThread Authors.
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
 *   This file contains implementation of ToBLE platform.
 */

#ifndef TOBLE_PLATFORM_HPP_
#define TOBLE_PLATFORM_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/toble.h>

#include "common/locator.hpp"
#include "common/string.hpp"

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

namespace ot {
namespace Toble {

/**
 * This class represents a ToBLE (BLE) address.
 *
 */
class Address : public otTobleAddress
{
public:
    enum
    {
        kInfoStringSize = 40, // Max chars for the info string (`ToString()`).
    };

    /**
     * This type defines the fixed-length `String` object returned from `ToString()`.
     *
     */
    typedef String<kInfoStringSize> InfoString;

    /**
     * This method evaluates whether or not two addresses match.
     *
     * @param[in]  aOther  Another address to compare with.
     *
     * @retval TRUE   If the two addresses match
     * @retval FALSE  If the two addresses do not match.
     *
     */
    bool operator==(const Address &aOther) const;

    /**
     * This method evaluates whether or not two addresses does not match.
     *
     * @param[in]  aOther  Another address to compare with.
     *
     * @retval TRUE   If the two addresses do not match.
     * @retval FALSE  If the two addresses match.
     *
     */
    bool operator!=(const Address &aOther) const;

    /**
     * This method converts an address to a string.
     *
     * @returns An `InfoString` containing the string representation of the Extended Address.
     *
     */
    InfoString ToString(void) const;
};

/**
 * This class represents an OpenThread ToBLE platform abstraction.
 *
 */
class Platform : public InstanceLocator
{
    friend class ot::Instance;

public:
    typedef otTobleAdvType          AdvType;
    typedef otTobleAdvConfig        AdvConfig;
    typedef otTobleConnection       Connection;
    typedef otTobleConnectionConfig ConnectionConfig;

    class Callbacks : public InstanceLocator
    {
        friend class Platform;

    public:
        void HandleConnected(Connection *aConn);
        void HandleDisconnected(Connection *aConn);
        void HandleConnectionReady(Connection *aConn, otTobleConnectionLinkType aLinkType);

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE
        void HandleAdv(AdvType aAdvType, const Address &aSource, const uint8_t *aData, uint16_t aLength, int8_t aRssi);
        void HandleC1WriteDone(Connection *aConn);
        void HandleC2Indication(Connection *aConn, const uint8_t *aBuf, uint16_t aLength);
#endif

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE
        void HandleC2Subscribed(Connection *aConn, bool aIsSubscribed);
        void HandleC2IndicateDone(Connection *aConn);
        void HandleC1Write(Connection *aConn, const uint8_t *aFrame, uint16_t aLength);
#endif

    private:
        explicit Callbacks(Instance &aInstance)
            : InstanceLocator(aInstance)
        {
        }
    };

    explicit Platform(Instance &aInstance)
        : InstanceLocator(aInstance)
        , mCallbacks(aInstance)
    {
    }

    // Common APIs

    void Init(void) { otPlatTobleInit(GetInstance()); }

    void Disconnect(Connection *aConn) { otPlatTobleDisconnect(GetInstance(), aConn); }

    uint16_t GetConnMtu(Connection *aConn) { return otPlatTobleGetMtu(GetInstance(), aConn); }

#if OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

    otError StartScan(uint16_t aInterval, uint16_t aWindow)
    {
        return otPlatTobleScanStart(GetInstance(), aInterval, aWindow);
    }

    otError StopScan(void) { return otPlatTobleScanStop(GetInstance()); }

    Connection *CreateConnection(const Address &aPeerAddress, ConnectionConfig &aConfig)
    {
        return otPlatTobleCreateConnection(GetInstance(), &aPeerAddress, &aConfig);
    }

    void WriteC1(Connection *aConn, const void *aBuf, uint16_t aLength)
    {
        otPlatTobleC1Write(GetInstance(), aConn, aBuf, aLength);
    }

    void SubscribeC2(Connection *aConn, bool aSubscribe)
    {
        return otPlatTobleC2Subscribe(GetInstance(), aConn, aSubscribe);
    }

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    //-- Add L2CAP platform APIs for central
#endif

#endif // OPENTHREAD_CONFIG_TOBLE_CENTRAL_ENABLE

#if OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

    otError StartAdv(const AdvConfig &aConfig) { return otPlatTobleAdvStart(GetInstance(), &aConfig); }
    otError StopAdv(void) { return otPlatTobleAdvStop(GetInstance()); }

    void IndicateC2(Connection *aConn, const void *aFrame, uint16_t aLength)
    {
        otPlatTobleC2Indicate(GetInstance(), aConn, aFrame, aLength);
    }

#if OPENTHREAD_CONFIG_TOBLE_L2CAP_ENABLE
    uint8_t GetL2capPsm(void) { return otPlatTobleGetL2capPsm(GetInstance()); }

    //-- Add other L2CAP platform APIs for peripheral
#endif

#endif // OPENTHREAD_CONFIG_TOBLE_PERIPHERAL_ENABLE

private:
    otInstance *GetInstance(void) { return reinterpret_cast<otInstance *>(&InstanceLocator::GetInstance()); }

    Callbacks mCallbacks;
};

} // namespace Toble
} // namespace ot

#endif // OPENTHREAD_CONFIG_ENABLE_TOBLE

#endif // TOBLE_PLATFORM_HPP_
