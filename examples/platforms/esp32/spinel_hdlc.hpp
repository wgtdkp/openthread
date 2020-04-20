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
 * This file defines HDLC spinel interface.
 *
 */

#ifndef OPENTHREAD_ESP32_SPINEL_HDLC_HPP_
#define OPENTHREAD_ESP32_SPINEL_HDLC_HPP_

#include "openthread-core-esp32-config.h"

#include <spinel_interface.hpp>

#include "openthread/openthread-system.h"

namespace otrtos {

namespace Platform {

namespace Esp32 {

typedef uint8_t HdlcSpinelContext;

/**
 * This class defines an HDLC spinel interface to the Radio Co-processor (RCP).
 *
 */
class HdlcInterface
{
public:
    /**
     * This constructor initializes the object.
     *
     * @param[in] aCallback         Callback on frame received
     * @param[in] aCallbackContext  Callback context
     * @param[in] aFrameBuffer      A reference to a `RxFrameBuffer` object.
     *
     */
    HdlcInterface(ot::Spinel::SpinelInterface::ReceiveFrameCallback aCallback,
                  void *                                        aCallbackContext,
                  ot::Spinel::SpinelInterface::RxFrameBuffer &      aFrameBuffer);

    /**
     * This destructor deinitializes the object.
     *
     */
    ~HdlcInterface(void);

    void Init(void);

    void Deinit(void);

    /**
     * This method encodes and sends a spinel frame to Radio Co-processor (RCP) over the socket.
     *
     * This is blocking call, i.e., if the socket is not writable, this method waits for it to become writable for
     * up to `kMaxWaitTime` interval.
     *
     * @param[in] aFrame     A pointer to buffer containing the spinel frame to send.
     * @param[in] aLength    The length (number of bytes) in the frame.
     *
     * @retval OT_ERROR_NONE     Successfully encoded and sent the spinel frame.
     * @retval OT_ERROR_NO_BUFS  Insufficient buffer space available to encode the frame.
     * @retval OT_ERROR_FAILED   Failed to send due to socket not becoming writable within `kMaxWaitTime`.
     *
     */
    otError SendFrame(const uint8_t *aFrame, uint16_t aLength);

    /**
     * This method waits for receiving part or all of spinel frame within specified interval.
     *
     * @param[in]  aTimeout  A reference to the timeout.
     *
     * @retval OT_ERROR_NONE             Part or all of spinel frame is received.
     * @retval OT_ERROR_RESPONSE_TIMEOUT No spinel frame is received within @p aTimeout.
     *
     */
    otError WaitForFrame(const struct timeval &aTimeout);

    /**
     * This method performs radio driver processing.
     *
     * @param[in]  aContext  The context containing no thing, never used.
     *
     */
    void Process(const otSysMainloopContext &aMainloop);

    void Update(otSysMainloopContext &aMainloop);

private:
    enum
    {
        kMaxFrameSize = ot::Spinel::SpinelInterface::kMaxFrameSize,
        kMaxWaitTime  = 2000, ///< Maximum wait time in Milliseconds for socket to become writable (see `SendFrame`).
    };

    void InitUart(void);
    void DeinitUart(void);

    int TryReadAndDecode(void);

    /**
     * This method waits for the UART file descriptor associated with the HDLC interface to become writable within
     * `kMaxWaitTime` interval.
     *
     * @retval OT_ERROR_NONE   Socket is writable.
     * @retval OT_ERROR_FAILED Socket did not become writable within `kMaxWaitTime`.
     *
     */
    otError WaitForWritable(void);

    /**
     * This method writes a given frame to the socket.
     *
     * This is blocking call, i.e., if the UART is not writable, this method waits for it to become writable for
     * up to `kMaxWaitTime` interval.
     *
     * @param[in] aFrame  A pointer to buffer containing the frame to write.
     * @param[in] aLength The length (number of bytes) in the frame.
     *
     * @retval OT_ERROR_NONE    Frame was written successfully.
     * @retval OT_ERROR_FAILED  Failed to write due to UART not becoming writable within `kMaxWaitTime`.
     *
     */
    otError Write(const uint8_t *aFrame, uint16_t aLength);

    static void HandleHdlcFrame(void *aContext, otError aError);
    void        HandleHdlcFrame(otError aError);

    ot::Spinel::SpinelInterface::ReceiveFrameCallback mReceiveFrameCallback;
    void *                                            mReceiveFrameContext;
    ot::Spinel::SpinelInterface::RxFrameBuffer &      mReceiveFrameBuffer;

    ot::Hdlc::Decoder mHdlcDecoder;
    uint8_t *mUartRxBuffer;

    int mUartFd;

    // Non-copyable, intentionally not implemented.
    HdlcInterface(const HdlcInterface &);
    HdlcInterface &operator=(const HdlcInterface &);
};

} // namespace Esp32

} // namespace Platform

} // namespace otrtos

#endif // OPENTHREAD_ESP32_SPINEL_HDLC_HPP_
