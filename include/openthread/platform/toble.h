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
 * @brief
 *   This file defines a TOBLE (Thread over BLE) platform APIs.
 *
 */

#ifndef OPENTHREAD_PLATFORM_TOBLE_H_
#define OPENTHREAD_PLATFORM_TOBLE_H_

#include <openthread/instance.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OT_PLAT_TOBLE_UUID_THREAD_GROUP 0xfffb

#define OT_PLAT_TOBLE_UUID_C1 \
    0x11, 0x9D, 0x9F, 0x42, 0x9C, 0x4F, 0x9F, 0x95, 0x59, 0x45, 0x3D, 0x26, 0xF5, 0x2E, 0xEE, 0x18

#define OT_PLAT_TOBLE_UUID_C2 \
    0x12, 0x9D, 0x9F, 0x42, 0x9C, 0x4F, 0x9F, 0x95, 0x59, 0x45, 0x3D, 0x26, 0xF5, 0x2E, 0xEE, 0x18

/**
 * This type represents different BLE address types
 *
 */
typedef enum otTobleAddressType
{
    OT_TOBLE_ADDRESS_TYPE_PUBLIC                        = 0, ///< Bluetooth public device address.
    OT_TOBLE_ADDRESS_TYPE_RANDOM_STATIC                 = 1, ///< Bluetooth random static address.
    OT_TOBLE_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE     = 2, ///< Bluetooth random private resolvable address.
    OT_TOBLE_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE = 3, ///< Bluetooth random private non-resolvable address.
} otTobleAddressType;

#define OT_TOBLE_ADDRESS_SIZE 6 ///< BLE address size (in bytes)

/**
 * This type represents a BLE address.
 *
 */
typedef struct otTobleAddress
{
    otTobleAddressType mType;                           ///< Bluetooth device address type.
    uint8_t            mAddress[OT_TOBLE_ADDRESS_SIZE]; ///< A 48-bit address of Bluetooth device in LSB format.
} otTobleAddress;

/**
 * This enumeration type represents BLE advertisement types.
 *
 */
typedef enum otTobleAdvType
{
    OT_TOBLE_ADV_IND         = 0x00, ///< Connectable undirected advertising.
    OT_TOBLE_ADV_DIRECT_IND  = 0x01, ///< Connectable directed advertising.
    OT_TOBLE_ADV_SCAN_IND    = 0x02, ///< Scanable undirected advertising.
    OT_TOBLE_ADV_NONCONN_IND = 0x03, ///< Non connectable undirected advertising.
} otTobleAdvType;

/**
 * This enumeration type represents BLE roles.
 *
 */
typedef enum otTobleRole
{
    OT_TOBLE_ROLE_UNKNOWN    = 0x00, ///< Unknown role.
    OT_TOBLE_ROLE_CENTRAL    = 0x01, ///< Central.
    OT_TOBLE_ROLE_PERIPHERAL = 0x02, ///< Peripheral.
} otTobleRole;

/**
 * Link type used by ToBLE connection.
 */
typedef enum otTobleConnectionLinkType
{
    kConnectionLinkTypeUnknown, ///< Type of connection is unknown yet
    kConnectionLinkTypeGatt,    ///< Connection uses GATT transport
    kConnectionLinkTypeL2Cap    ///< Connection uses L2CAP transport
} otTobleConnectionLinkType;

typedef uint8_t otTobleConnectionId; ///< An identifier representing ToBLE connection.

#define OT_TOBLE_ADV_DATA_MAX_LENGTH 31     ///< Maximum length of advertising data [bytes].
#define OT_TOBLE_CONNECTION_ID_INVALID 0xff ///< Invalid ToBLE connection identifier.

/**
 * This type represents advertisement configuration.
 *
 */
typedef struct otTobleAdvConfig
{
    otTobleAdvType mType;     ///< Advertisement type.
    uint16_t       mInterval; ///< Advertisement interval (in ms).
    const uint8_t *mData;     ///< Advertisement data - Formatted as sequence of "<len, type, data>" structures.
    uint16_t       mLength;   ///< Advertisement data length (number of bytes).
} otTobleAdvConfig;

/**
 * This type represents configuration for creating a new connection
 *
 */
typedef struct otTobleConnectionConfig
{
    uint16_t mInterval;     ///< Connection interval (in ms).
    uint16_t mScanInterval; ///< Scan interval while connecting (in ms).
    uint16_t mScanWindow;   ///< Scan window while connection (in ms).

    otTobleConnectionLinkType mLinkType; ///< Which link type should be used for this connection.
} otTobleConnectionConfig;

/**
 * This function initializes ToBLE platform.
 *
 * It should be called before any other ToBLE platform function.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 *
 */
void otPlatTobleInit(otInstance *aInstance);

/**
 * This function disconnects a connection.
 *
 * If the connection is already established, this function disconnects it. If the connection is being established
 * it stops the connect attempts.
 *
 * After this call, the `aConn` should be considered invalid and not used again.
 *
 * NOTE: When a connection is disconnected using an explicit call to this function, the platform should NOT invoke the
 * callback `otPlatTobleHandleDisconnected()`.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     The connection disconnect
 *
 */
void otPlatTobleDisconnect(otInstance *aInstance, otTobleConnectionId aConnId);

/**
 * This is a callback to indicate that a connection is established.
 *
 * On a central device, this is called when connection is established after a successful call to
 * `otPlatTobleCreateConnection()`. On a peripheral device, this is called when a central connects to the device
 * while device is sending connectable advertisements.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 *
 */
extern void otPlatTobleHandleConnected(otInstance *aInstance, otTobleConnectionId aConnId);

/**
 * This is a callback to indicate that a previously connected connection got disconnected.
 *
 * After this call, the `aConn` should be considered invalid and not used again.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 *
 */
extern void otPlatTobleHandleDisconnected(otInstance *aInstance, otTobleConnectionId aConnId);

/**
 * This functions get the MTU size of a connection.
 *
 * On central, this can be called only after callback `otPlatTobleHandleConnectionIsReady()` is received.
 * On peripheral, this can be called only after callback `otPlatTobleHandleC2Subscribed()` is received.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 *
 * @returns The MTU size of connection.
 *
 */
uint16_t otPlatTobleGetMtu(otInstance *aInstance, otTobleConnectionId aConnId);

/*----------------------------------------------------------------------------------------------------------------------
 * Central APIs
 */

/**
 * This function requests scanning to start.
 *
 * A scan start request while a connection is being established should fail and return error. OpenThread ToBLE code
 * ensures to not call this function while a connection is being established.
 *
 * @param[in] aInstance    A pointer to OpenThread instance.
 * @param[in] aInterval    The scan interval (in ms).
 * @param[in] aWindow      The scan window (in ms).
 *
 * @retval OT_ERROR_NONE            Started scanning.
 * @retval OT_ERROR_INVALID_STATE   Scan could not start (in middle of connecting).
 *
 */
otError otPlatTobleScanStart(otInstance *aInstance, uint16_t aInterval, uint16_t aWindow);

/**
 * This function request scanning to be stopped.
 *
 * NOTE: OpenThread TobLE code may call `otPlatTobleScanStop()` while there is no ongoing scan.
 *
 * @param[in] aInstance  A pointer to OpenThread instance.
 *
 * @retval OT_ERROR_NONE            Stopped scanning (or there was no ongoing scan).
 *
 */
otError otPlatTobleScanStop(otInstance *aInstance);

/**
 * This is a callback to indicate an advertisement beacon was received (during scanning).
 *
 * @param[in] aInstance    A pointer to OpenThread instance.
 * @param[in] aAdvType     The advertisement type.
 * @param[in] aSource      The advertisement source address.
 * @param[in] aData        The advertisement data - sequence of "<len, type, data>" structures.
 * @param[in] aLength      The advertisement data length.
 * @param[in] aRssi        The RSSI of the received advertisement (or 127 if not available).
 *
 */
extern void otPlatTobleHandleAdv(otInstance *          aInstance,
                                 otTobleAdvType        aAdvType,
                                 const otTobleAddress *aSource,
                                 const uint8_t *       aData,
                                 uint16_t              aLength,
                                 int8_t                aRssi);
/**
 * This function creates a new connection to given address/peripheral and starts establishing the connection.
 *
 * This should be called when there is no ongoing scan (i.e., any previous scan using `otPlatTobleScanStart()` should
 * be stopped using `otPlatTobleScanStop()` before calling this function to create a new connection).
 *
 * The connection request is one-time use: once connected and disconnected, this function should be called again to
 * create a connection to the same peer.
 *
 * On success, this function returns a valid connection identifeir. After this, `otPlatTobleHandleConnected()` callback
 * is used by platform to notify when connection is established. Platform should not provide a timeout, i.e., it is
 * expected that platform continues to try to connect to the peer, until it either successfully connects (and invokes
 * the `otPlatTobleHandleConnected()`) or told to disconnect (`otPlatTobleDisconnect` is called).
 *
 * @param[in] aInstance      A pointer to OpenThread instance.
 * @param[in] aPeerAddress   The peer to connect to.
 * @param[in] aConfig        The connection configuration.
 *
 * @returns On success an identifier for the created connection, or OT_TOBLE_CONNECTION_ID_INVALID on failure.
 *
 */
otTobleConnectionId otPlatTobleCreateConnection(otInstance *             aInstance,
                                                const otTobleAddress *   aPeerAddress,
                                                otTobleConnectionConfig *aConfig);

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * Central APIs for BTP transport support
 *
 */

/**
 * This is a callback to indicate that connection is ready.
 *
 * The platform should call this after `otPlatTobleHandleConnected()` to indicate the connection is ready for use.
 * Only after getting this call, we can use any of the other functions that interact with the connection (e.g.
 * `otPlatTobleGetMtu()` or `otPlatTobleC1Write(), etc. - all functions that have `aConnId` as input parameter). The
 * only exception to this is the `otPlatTobleDisconnect()` which can be called any time after connection is created.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 * @param[in] aLinkType   A connection link type which is used as low level transport.
 *
 */
extern void otPlatTobleHandleConnectionIsReady(otInstance *              aInstance,
                                               otTobleConnectionId       aConnId,
                                               otTobleConnectionLinkType aLinkType);

/**
 * This function requests a C1 write (BTP) on a given connection.
 *
 * The platform notifies that the write request is done by calling `otPlatTobleHandleC1WriteDone()`.
 *
 * This can be called only after callback `otPlatTobleHandleConnectionIsReady()` is received for the connection.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 * @param[in] aBuffer     A pointer to buffer to write.
 * @param[in] aLength     Length of buffer (number of bytes).
 *
 */
void otPlatTobleC1Write(otInstance *aInstance, otTobleConnectionId aConnId, const void *aBuffer, uint16_t aLength);

/**
 * This is a callback to notify that a C1 write request was done.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 *
 */
extern void otPlatTobleHandleC1WriteDone(otInstance *aInstance, otTobleConnectionId aConnId);

/**
 * This function requests a subscription change to C2 (BTP) on a given connection.
 *
 * NOTE: There is no callback for done for this request.
 *
 * This can be called only after callback `otPlatTobleHandleConnectionIsReady()` is received for the connection.
 *
 * @param[in] aInstance    A pointer to OpenThread instance.
 * @param[in] aConnId      ToBLE connection identifier.
 * @param[in] aSubscrive   TRUE to subscribe, FALSE to un-subscribe.
 *
 */
void otPlatTobleC2Subscribe(otInstance *aInstance, otTobleConnectionId aConnId, bool aSubscribe);

/**
 * This is callback to notify that peer has indicated C2 (BTP) on a given connection.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 * @param[in] aBuffer     A pointer to buffer (from C2 indication).
 * @param[in] aLength     Length of buffer (number of bytes).
 *
 */
extern void otPlatTobleHandleC2Indication(otInstance *        aInstance,
                                          otTobleConnectionId aConnId,
                                          const uint8_t *     aBuffer,
                                          uint16_t            aLength);

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * Central APIs for L2CAP transport support
 *
 */

//-- TO BE DEFINED --

/*----------------------------------------------------------------------------------------------------------------------
 * Peripheral APIs
 */

/**
 * This function start transmission of advertisements.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConfig     A pointer to advertisement configuration.
 *
 * @retval OT_ERROR_NONE   Started successfully.
 *
 */
otError otPlatTobleAdvStart(otInstance *aInstance, const otTobleAdvConfig *aConfig);

/**
 * This function stops transmission of advertisements.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 *
 * @retval OT_ERROR_NONE   Stopped successfully.
 *
 */
otError otPlatTobleAdvStop(otInstance *aInstance);

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * Peripheral APIs for BTP transport
 *
 */

/**
 * This is callback to notify that peer has subscribed to C2 (BTP) on a connection.
 *
 * @param[in] aInstance       A pointer to OpenThread instance.
 * @param[in] aConnId         A pointer to the connection.
 * @param[in] aIsSubscribed   TRUE if the peer subscribed, FALSE if un-subscribed.
 *
 */
extern void otPlatTobleHandleC2Subscribed(otInstance *aInstance, otTobleConnectionId aConnId, bool aIsSubscribed);

/**
 * This function request a C2 indicate on a connection.
 *
 * The platform notifies that the indicate request is done by calling `otPlatTobleHandleC2IndicateDone()`.
 *
 * This can be called only after `otPlatTobleHandleC2Subscribed() callback is received on the same connection.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 * @param[in] aBuffer     A pointer to buffer (from C2 indication)
 * @param[in] aLength     Length of buffer (number of bytes).
 *
 */
void otPlatTobleC2Indicate(otInstance *aInstance, otTobleConnectionId aConnId, const void *aBuffer, uint16_t aLength);

/**
 * This is a callback to notify that a C2 indicate request was done.
 *
 * @param[in] aInstance  A pointer to OpenThread instance.
 * @param[in] aConnId    ToBLE connection identifier.
 *
 */
extern void otPlatTobleHandleC2IndicateDone(otInstance *aInstance, otTobleConnectionId aConnId);

/**
 * This is callback to notify that peer has written to C1 (BTP) on a given connection.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 * @param[in] aBuffer     A pointer to buffer (from C1 write.)
 * @param[in] aLength     Length of buffer (number of bytes).
 *
 */
extern void otPlatTobleHandleC1Write(otInstance *        aInstance,
                                     otTobleConnectionId aConnId,
                                     const uint8_t *     aBuffer,
                                     uint16_t            aLength);

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * Peripheral APIs for L2CAP transport support
 *
 */

/**
 * This function requests the ToBLE platform layer to provide the L2CAP PSM value.
 *
 */
uint8_t otPlatTobleGetL2capPsm(otInstance *aInstance);

/**
 *  This function returns an underlying link type of platform connection.
 *
 * @returns Type of the link, if no connection is established kConnectionLinkTypeUnknown is returned.
 *
 */
otTobleConnectionLinkType otPlatTobleGetConnectionLinkType(const otTobleConnectionId aConnId);

/**
 * This is a callback to notify that new data frame came through the L2Cap connection.
 *
 * Note: Ownership of the aBuffer buffer is not passed. Its content may change after returning from the callback. Do not
 * cache this pointer.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 * @param[in] aBuffer     A pointer to incoming buffer
 * @param[in] aLength     Length of buffer (number of bytes).
 *
 */
void otPlatTobleL2CapFrameReceived(otInstance *        aInstance,
                                   otTobleConnectionId aConnId,
                                   const uint8_t *     aBuffer,
                                   uint16_t            aLength);

/**
 * This function sends a packet through L2Cap link layer.
 *
 * This function this can be called only after callback `otPlatTobleHandleConnectionIsReady()`is received with link type
 * indication L2Cap transport for `aConnId`.
 *
 * The platform notifies that the send request is done by calling `otPlatBleL2capOnSduSent()`.
 *
 * @param[in] aInstance   A pointer to OpenThread instance.
 * @param[in] aConnId     ToBLE connection identifier.
 * @param[in] aBuffer     A pointer to buffer which will be sent.
 * @param[in] aLength     Length of buffer (number of bytes).
 *
 * @retval OT_ERROR_NOT_FOUND       If aConnId == NULL.
 * @retval OT_ERROR_NONE            LE Credit Based Connection Request has been sent.
 * @retval OT_ERROR_INVALID_STATE   BLE Device is in an invalid state e.g. not in the GAP connection.
 * @retval OT_ERROR_INVALID_ARGS    Invalid parameters have been provided.
 * @retval OT_ERROR_NO_BUFS         No available internal buffer found.
 *
 */
extern otError otPlatTobleL2capSend(otInstance *        aInstance,
                                    otTobleConnectionId aConnId,
                                    const uint8_t *     aBuffer,
                                    uint16_t            aLength);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_PLATFORM_TOBLE_H_
