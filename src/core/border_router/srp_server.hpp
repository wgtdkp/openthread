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
 *   This file includes definitions for SRP server.
 */

#ifndef BORDER_ROUTER_SRP_SERVER_HPP_
#define BORDER_ROUTER_SRP_SERVER_HPP_

#include "openthread-core-config.h"

#if OPENTHREAD_CONFIG_SRP_SERVER_ENABLE

#include <openthread/ip6.h>
#include <openthread/srp.h>

#include "common/clearable.hpp"
#include "common/linked_list.hpp"
#include "common/locator.hpp"
#include "common/non_copyable.hpp"
#include "common/notifier.hpp"
#include "common/timer.hpp"
#include "net/dns_headers.hpp"
#include "net/ip6_address.hpp"
#include "net/udp6.hpp"

namespace ot {

namespace BorderRouter {

/**
 * This class implements the SRP server.
 *
 */
class SrpServer : public InstanceLocator, private NonCopyable
{
    friend class ot::Notifier;

public:
    class Host;
    class Key;
    class Service;

    enum : uint8_t
    {
        kMaxAddressesNum = OPENTHREAD_CONFIG_IP6_SLAAC_NUM_ADDRESSES, ///< The maximum addresses number for a host.
    };

    explicit SrpServer(Instance &aInstance);

    void SetServiceHandler(otSrpServiceHandler aServiceHandler, void *aServiceHandlerContext);

    void    SetEnabled(bool aEnabled);
    otError SetLeaseRange(uint32_t aMinLease, uint32_t aMaxLease, uint32_t aMinKeyLease, uint32_t aMaxKeyLease);

    uint32_t GrantLease(uint32_t aLease) const;
    uint32_t GrantKeyLease(uint32_t aKeyLease) const;

    const Host *GetNextHost(const Host *aHost);

    void HandleServiceEventResult(otError aError, void *aUpdate);
    void HandleServiceEventResult(otError                 aError,
                                  const Dns::Header &     aDnsHeader,
                                  Host &                  aHost,
                                  const Ip6::MessageInfo &aMessageInfo);

    /**
     * This class represents a SRP service.
     *
     */
    class Service : public otSrpService, public LinkedListEntry<Service>, private NonCopyable
    {
    public:
        static Service *New(const char *aName, uint8_t aNameLength);
        static void     Destroy(Service *aService);

        bool IsDeleted(void) const { return (GetHost().IsDeleted() || mDeleted == 0); }

        TimeMilli GetExpireTime(void) const;
        TimeMilli GetKeyExpireTime(void) const;

        const Host &GetHost(void) const { return *static_cast<Host *>(mHost); }
        Host &      GetHost(void) { return *static_cast<Host *>(mHost); }

        const char *GetFullName() const { return mFullName; }
        char *      GetFullName() { return mFullName; }
        otError     SetFullName(const char *aName, uint8_t aLength);

        Service *      GetNext(void) { return static_cast<Service *>(mNext); }
        const Service *GetNext(void) const { return static_cast<const Service *>(mNext); }

        void    SetPort(uint16_t aPort) { mPort = aPort; }
        otError SetTxtData(const uint8_t *aTxtData, uint16_t aTxtDataLength);
        otError SetTxtDataFromMessage(const Message &aMessage, uint16_t aOffset, uint16_t aLength);

        void    ClearResources(void);
        otError CopyResourcesFrom(const Service &aService);

        bool Matches(const char *aName) const;

        bool operator==(const Service &aOther) const;
        bool operator!=(const Service &aOther) const { return !(*this == aOther); }

    private:
        Service();
        ~Service();

        TimeMilli mTimeLastUpdate;
    };

    /**
     * This class represents a Host which registers one or more services on this SRP server.
     *
     */
    class Host : public LinkedListEntry<Host>, private NonCopyable
    {
    public:
        friend class LinkedListEntry<Host>;

        static Host *New();
        static void  Destroy(Host *aHost);

        bool IsDeleted(void) const { return (mLease == 0); }

        const char *GetFullName() const { return mFullName; }
        char *      GetFullName() { return mFullName; }
        otError     SetFullName(const char *aName, uint8_t aLength);

        const Ip6::Address *GetAddresses(uint8_t &aAddressesNum) const
        {
            aAddressesNum = mAddressesNum;
            return mAddresses;
        }

        void     SetLease(uint32_t aLease);
        uint32_t GetLease(void) const { return mLease; }

        void     SetKeyLease(uint32_t aKeyLease);
        uint32_t GetKeyLease(void) const { return mKeyLease; }

        const Key *GetKey(void) const { return mKey; }

        TimeMilli      GetExpireTime(void) const;
        TimeMilli      GetKeyExpireTime(void) const;
        const Service *GetServices() const { return mServices.GetHead(); }
        Service *      GetServices() { return mServices.GetHead(); }

        // Add service. Do nothing if there is existing one.
        Service *AddService(const char *aName, uint16_t aNameLength);
        Service *AddService(const char *aName) { return AddService(aName, strlen(aName)); }

        // Add a service.
        void AddService(Service *aService);
        void RemoveService(Service *aService);
        void RemoveAllServices(void);

        /**
         * This method clears resources of this host and all service registered by this host.
         *
         */
        void     ClearResources(void);
        void     CopyResourcesFrom(const Host &aHost);
        Service *FindService(const char *aFullName);

        otError AddIp6Address(const Ip6::Address &aIp6Address);

        bool Matches(const char *aName) const;

        bool operator==(const Host &aOther) const;
        bool operator!=(const Host &aOther) const { return !(*this == aOther); }

    private:
        Host();
        ~Host();

        char *       mFullName;                    // The full hostname.
        Ip6::Address mAddresses[kMaxAddressesNum]; // The address list to be advertised.
        uint8_t      mAddressesNum;                // The number of addresses.
        Host *       mNext;

        Key *               mKey;
        uint32_t            mLease;
        uint32_t            mKeyLease;
        TimeMilli           mTimeLastUpdate;
        LinkedList<Service> mServices;
    };

    /**
     * This class implements the KEY Resource.
     */
    class Key
    {
    public:
        static SrpServer::Key *New();
        static void            Destroy(Key *aKey);

        otError SetKeyFromMessage(const Message &aMessage, uint16_t aOffset, uint16_t aLength);
        otError ValidateSignature(const uint8_t *aSig, uint16_t aSigLength);

        bool operator==(const Key &aOther) const;
        bool operator!=(const Key &aOther) const { return !(*this == aOther); }

    private:
        Key();
        ~Key();

        uint16_t mFlags;
        uint8_t  mProtocol;
        uint8_t  mAlgorithm;
        uint8_t *mKey;
        uint16_t mKeyLength;
    };

private:
    enum : uint8_t
    {
        kThreadServiceTypeSrpServer = 0x5du,
    };

    enum : uint32_t
    {
        kThreadEnterpriseNumber      = 44970u,
        kDefaultMinLease             = 1800u,         // Default minimum lease time, 30 min.
        kDefaultMaxLease             = 7200u,         // Default maximum lease time, 2 hours.
        kDefaultMinKeyLease          = 3600 * 24,     // Default minimum key-lease time, 1 day.
        kDefaultMaxKeyLease          = 3600 * 24 * 7, // Default maximum key-lease time, 14 days.
        kDefaultEventsHandlerTimeout = 500,           // Default events handler timeout in milliseconds.
    };

    /**
     * This class includes metadata for processing a SRP update (register, deregister)
     * and sending DNS response to the client.
     *
     */
    class UpdateMetadata : public LinkedListEntry<UpdateMetadata>
    {
    public:
        friend class LinkedListEntry<UpdateMetadata>;

        static UpdateMetadata *New(const Dns::Header &aHeader, Host *aHost, const Ip6::MessageInfo &aMessageInfo);
        static void            Destroy(UpdateMetadata *aUpdateMetadata);

        TimeMilli               GetExpireTime(void) const { return mExpireTime; }
        const Dns::Header &     GetDnsHeader(void) const { return mDnsHeader; }
        Host *                  GetHost(void) { return mHost; }
        const Ip6::MessageInfo &GetMessageInfo(void) const { return mMessageInfo; }

        bool Matches(uint16_t aMessageId) const { return mDnsHeader.GetMessageId() == aMessageId; }

    private:
        UpdateMetadata(const Dns::Header &aHeader, Host *aHost, const Ip6::MessageInfo &aMessageInfo);
        ~UpdateMetadata() = default;

        TimeMilli        mExpireTime;  // Expire time of this update; In milliseconds.
        Dns::Header      mDnsHeader;   // The header of the DNS update request.
        Host *           mHost;        // The host will be updated. The UpdateMetadata has no ownership of this host.
        Ip6::MessageInfo mMessageInfo; // The message info of the DNS update request.
        UpdateMetadata * mNext;
    };

    void Start();

    void Stop();

    void HandleNotifierEvents(Events aEvents);

    otError PublishService();
    void    UnpublishService();

    /**
     * This method returns the smallest OMR address.
     *
     * @returns  The smallest OMR prefix if there is; Otherwise, NULL.
     *
     */
    const Ip6::Address *GetSmallestOmrAddress() const;

    void HandleDnsUpdate(Message &               aMessage,
                         const Ip6::MessageInfo &aMessageInfo,
                         const Dns::Header &     aDnsHeader,
                         uint16_t                aOffset);

    otError ProcessZoneSection(const Message &    aMessage,
                               const Dns::Header &aDnsHeader,
                               uint16_t &         aOffset,
                               Dns::Zone &        aZone);
    otError ProcessUpdateSection(const Message &         aMessage,
                                 const Ip6::MessageInfo &aMessageInfo,
                                 const Dns::Header &     aDnsHeader,
                                 const Dns::Zone &       aZone,
                                 uint16_t                aHeaderOffset,
                                 uint16_t &              aOffset);
    otError ProcessAdditionalSection(const Message &    aMessage,
                                     const Dns::Header &aDnsHeader,
                                     uint16_t           aHeaderOffset,
                                     uint16_t &         aOffset,
                                     Host *             aHost);

    /**
     * This method handles the update.
     *
     */
    void HandleUpdate(const Dns::Header &aDnsHeader, Host *aHost, const Ip6::MessageInfo &aMessageInfo);

    void TriggerUpdateEvent(const Dns::Header &aDnsHeader, Host *aHost, const Ip6::MessageInfo &aMessageInfo);
    void CompleteUpdate(const Dns::Header &aDnsHeader, Host *aHost, const Ip6::MessageInfo &aMessageInfo);

    const Host *GetHost(const char *aHostName);

    /**
     * This method adds a SRP service host and takes ownership of it.
     *
     * The caller MUST make sure that there is no existing host with the same hostname.
     *
     */
    void AddHost(Host *aHost);

    /**
     * This method removes and destroyes the host.
     *
     */
    void RemoveHost(Host *aHost);

    Service *FindService(const char *aFullName);

    bool HasNameConflictsWith(Host *aHost);

    void SendResponse(const Dns::Header &     aHeader,
                      Dns::Header::Response   aResponseCode,
                      const Ip6::MessageInfo &aMessageInfo);
    void SendResponse(const Dns::Header &     aHeader,
                      uint32_t                aLease,
                      uint32_t                aKeyLease,
                      const Ip6::MessageInfo &aMessageInfo);

    static void HandleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo);
    void        HandleUdpReceive(Message &aMessage, const Ip6::MessageInfo &aMessageInfo);

    static void HandleLeaseTimer(Timer &aTimer);
    void        HandleLeaseTimer(void);

    static void HandleOutstandingUpdatesTimer(Timer &aTimer);
    void        HandleOutstandingUpdatesTimer(void);

    bool mEnabled;

    Ip6::Udp::Socket    mSocket;
    otSrpServiceHandler mServiceHandler;
    void *              mServiceHandlerContext;

    uint32_t mMinLease;    // The minimum lease time in seconds.
    uint32_t mMaxLease;    // The maximum lease time in seconds.
    uint32_t mMinKeyLease; // The minimum key-lease time in seconds.
    uint32_t mMaxKeyLease; // The maximum key-lease time in seconds.

    Ip6::Address mPublishedAddress;

    LinkedList<Host> mHosts;
    TimerMilli       mLeaseTimer;

    TimerMilli                 mOutstandingUpdatesTimer;
    LinkedList<UpdateMetadata> mOutstandingUpdates;
};

} // namespace BorderRouter

} // namespace ot

#endif // OPENTHREAD_CONFIG_SRP_SERVER_ENABLE

#endif // BORDER_ROUTER_SRP_SERVER_HPP_
