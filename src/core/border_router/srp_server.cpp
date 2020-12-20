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
 *   This file includes implementation for SRP server.
 */

#include "srp_server.hpp"

#if OPENTHREAD_CONFIG_SRP_SERVER_ENABLE

#include "border_router/routing_manager.hpp"
#include "common/instance.hpp"
#include "common/locator-getters.hpp"
#include "common/logging.hpp"
#include "common/new.hpp"
#include "net/dns_headers.hpp"
#include "thread/network_data_local.hpp"
#include "thread/network_data_notifier.hpp"
#include "thread/thread_netif.hpp"
#include "utils/heap.hpp"

namespace ot {

namespace BorderRouter {

static Dns::Header::Response ErrorToResponseCode(otError aError)
{
    Dns::Header::Response responseCode;

    switch (aError)
    {
    case OT_ERROR_NONE:
        responseCode = Dns::Header::kResponseSuccess;
        break;
    case OT_ERROR_NO_BUFS:
        responseCode = Dns::Header::kResponseServerFailure;
        break;
    case OT_ERROR_PARSE:
        responseCode = Dns::Header::kResponseFormatError;
        break;
    default:
        responseCode = Dns::Header::kResponseRefused;
        break;
    }

    return responseCode;
}

SrpServer::SrpServer(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mEnabled(true) // The SRP server is enabled by default.
    , mSocket(aInstance)
    , mServiceHandler(nullptr)
    , mServiceHandlerContext(nullptr)
    , mMinLease(kDefaultMinLease)
    , mMaxLease(kDefaultMaxLease)
    , mMinKeyLease(kDefaultMinKeyLease)
    , mMaxKeyLease(kDefaultMaxKeyLease)
    , mLeaseTimer(aInstance, HandleLeaseTimer, this)
    , mOutstandingUpdatesTimer(aInstance, HandleOutstandingUpdatesTimer, this)
{
    mPublishedAddress.Clear();
}

void SrpServer::SetServiceHandler(otSrpServiceHandler aServiceHandler, void *aServiceHandlerContext)
{
    mServiceHandler        = aServiceHandler;
    mServiceHandlerContext = aServiceHandlerContext;
}

void SrpServer::SetEnabled(bool aEnabled)
{
    if (aEnabled && !mEnabled)
    {
        if (Get<Mle::MleRouter>().IsAttached())
        {
            Start();
        }
    }
    else if (!aEnabled && mEnabled)
    {
        Stop();
    }

    mEnabled = aEnabled;
}

otError SrpServer::SetLeaseRange(uint32_t aMinLease, uint32_t aMaxLease, uint32_t aMinKeyLease, uint32_t aMaxKeyLease)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(aMinLease <= aMaxLease, error = OT_ERROR_INVALID_ARGS);
    VerifyOrExit(aMinKeyLease <= aMaxKeyLease, error = OT_ERROR_INVALID_ARGS);
    VerifyOrExit(aMinLease <= aMinKeyLease, error = OT_ERROR_INVALID_ARGS);
    VerifyOrExit(aMaxLease <= aMaxKeyLease, error = OT_ERROR_INVALID_ARGS);

    mMinLease    = aMinLease;
    mMaxLease    = aMaxLease;
    mMinKeyLease = aMinKeyLease;
    mMaxKeyLease = aMaxKeyLease;

exit:
    return error;
}

uint32_t SrpServer::GrantLease(uint32_t aLease) const
{
    OT_ASSERT(mMinLease <= mMaxLease);

    return (aLease == 0) ? 0 : OT_MAX(mMinLease, OT_MIN(mMaxLease, aLease));
}

uint32_t SrpServer::GrantKeyLease(uint32_t aKeyLease) const
{
    OT_ASSERT(mMinKeyLease <= mMaxKeyLease);

    return (aKeyLease == 0) ? 0 : OT_MAX(mMinKeyLease, OT_MIN(mMaxKeyLease, aKeyLease));
}

const SrpServer::Host *SrpServer::GetNextHost(const SrpServer::Host *aHost)
{
    return (aHost == nullptr) ? mHosts.GetHead() : aHost->GetNext();
}

const SrpServer::Host *SrpServer::GetHost(const char *aHostName)
{
    Host *ret = nullptr;

    for (Host *host = mHosts.GetHead(); host != nullptr; host = host->GetNext())
    {
        if (StringStartsWith(host->GetFullName(), aHostName))
        {
            ret = host;
            break;
        }
    }

    return ret;
}

void SrpServer::AddHost(Host *aHost)
{
    otError error;

    OT_ASSERT(mHosts.FindMatching(aHost->GetFullName()) == nullptr);

    IgnoreError(error = mHosts.Add(*aHost));
    OT_ASSERT(error == OT_ERROR_NONE);
}

void SrpServer::RemoveHost(Host *aHost)
{
    otLogInfoBr("SRP server: permanently removes host %s", aHost->GetFullName());
    mHosts.Remove(*aHost);
    Host::Destroy(aHost);
}

SrpServer::Service *SrpServer::FindService(const char *aFullName)
{
    Service *service = nullptr;

    for (auto host = mHosts.GetHead(); host != nullptr; host = host->GetNext())
    {
        service = host->FindService(aFullName);
        if (service != nullptr)
        {
            break;
        }
    }

    return service;
}

bool SrpServer::HasNameConflictsWith(Host *aHost)
{
    bool hasConflicts = false;
    auto existingHost = mHosts.FindMatching(aHost->GetFullName());

    if (existingHost != nullptr && *aHost->GetKey() != *existingHost->GetKey())
    {
        ExitNow(hasConflicts = true);
    }

    for (auto service = aHost->GetServices(); service != nullptr; service = service->GetNext())
    {
        auto existingService = FindService(service->GetFullName());
        if (existingService != nullptr && *service->GetHost().GetKey() != *existingService->GetHost().GetKey())
        {
            ExitNow(hasConflicts = true);
        }
    }

exit:
    return hasConflicts;
}

void SrpServer::HandleServiceEventResult(otError aError, void *aUpdate)
{
    auto update = static_cast<UpdateMetadata *>(aUpdate);

    HandleServiceEventResult(aError, update->GetDnsHeader(), *update->GetHost(), update->GetMessageInfo());

    IgnoreError(mOutstandingUpdates.Remove(*update));
    UpdateMetadata::Destroy(update);

    if (mOutstandingUpdates.IsEmpty())
    {
        mOutstandingUpdatesTimer.Stop();
    }
    else
    {
        mOutstandingUpdatesTimer.StartAt(mOutstandingUpdates.GetTail()->GetExpireTime(), 0);
    }
}

void SrpServer::HandleServiceEventResult(otError                 aError,
                                         const Dns::Header &     aDnsHeader,
                                         Host &                  aHost,
                                         const Ip6::MessageInfo &aMessageInfo)
{
    Host *   existingHost;
    uint32_t hostLease;
    uint32_t hostKeyLease;
    uint32_t grantedLease;
    uint32_t grantedKeyLease;

    SuccessOrExit(aError);

    hostLease       = aHost.GetLease();
    hostKeyLease    = aHost.GetKeyLease();
    grantedLease    = GrantLease(hostLease);
    grantedKeyLease = GrantKeyLease(hostKeyLease);

    aHost.SetLease(grantedLease);
    aHost.SetKeyLease(grantedKeyLease);

    existingHost = mHosts.FindMatching(aHost.GetFullName());

    if (aHost.GetLease() == 0)
    {
        otLogInfoBr("SRP server removes host %s", aHost.GetFullName());

        if (aHost.GetKeyLease() == 0)
        {
            otLogInfoBr("SRP server removes key of host %s", aHost.GetFullName());

            if (existingHost != nullptr)
            {
                RemoveHost(existingHost);
            }
        }
        else if (existingHost != nullptr)
        {
            existingHost->SetLease(aHost.GetLease());
            existingHost->SetKeyLease(aHost.GetKeyLease());

            // Clear all resources associated to this host and services.
            existingHost->ClearResources();
            for (auto service = existingHost->GetServices(); service != nullptr; service = service->GetNext())
            {
                service->ClearResources();
            }
        }

        Host::Destroy(&aHost);
    }
    else if (existingHost != nullptr)
    {
        // Merge current updates into existing host.

        otLogInfoBr("SRP server updates host %s", existingHost->GetFullName());

        existingHost->CopyResourcesFrom(aHost);

        for (auto service = aHost.GetServices(); service != nullptr; service = service->GetNext())
        {
            auto existingService = existingHost->AddService(service->GetFullName());

            VerifyOrExit(existingService != nullptr, aError = OT_ERROR_NO_BUFS);
            SuccessOrExit(aError = existingService->CopyResourcesFrom(*service));

            otLogInfoBr("SRP server adds/updates service %s", existingHost->GetFullName());
        }

        Host::Destroy(&aHost);
    }
    else
    {
        otLogInfoBr("SRP server adds new host %s", aHost.GetFullName());

        // Add current host in the SRP update if this is the first update of it.
        AddHost(&aHost);
    }

    // Restart or stop the lease timer.
    HandleLeaseTimer();

exit:
    if (aError == OT_ERROR_NONE && !(grantedLease == hostLease && grantedKeyLease == hostKeyLease))
    {
        SendResponse(aDnsHeader, grantedLease, grantedKeyLease, aMessageInfo);
    }
    else
    {
        SendResponse(aDnsHeader, ErrorToResponseCode(aError), aMessageInfo);
    }
}

void SrpServer::Start()
{
    otError error;

    OT_ASSERT(!mPublishedAddress.IsUnspecified());

    SuccessOrExit(error = mSocket.Open(HandleUdpReceive, this));
    SuccessOrExit(error = mSocket.Bind(Ip6::SockAddr(mPublishedAddress, 0)));

    SuccessOrExit(error = PublishService());

    otLogInfoBr("SRP server starts listening on %s", mSocket.GetSockName().ToString().AsCString());

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogCritBr("failed to start SRP server: %s", otThreadErrorToString(error));
        // Cleanup any resources we may have allocated.
        Stop();
    }
}

void SrpServer::Stop()
{
    otLogInfoBr("SRP server stops listening on %s", mSocket.GetSockName().ToString().AsCString());

    UnpublishService();

    IgnoreError(mSocket.Close());
}

void SrpServer::HandleNotifierEvents(Events aEvents)
{
    VerifyOrExit(mEnabled);

    if (aEvents.Contains(kEventIp6AddressAdded) || aEvents.Contains(kEventIp6AddressRemoved))
    {
        if (mPublishedAddress.IsUnspecified())
        {
            const Ip6::Address *omrAddress = GetSmallestOmrAddress();
            if (omrAddress != nullptr)
            {
                mPublishedAddress = *omrAddress;
                Start();
            }
        }
        else
        {
            if (!Get<ThreadNetif>().HasUnicastAddress(mPublishedAddress))
            {
                Stop();
                mPublishedAddress.Clear();
            }
        }
    }

exit:
    return;
}

const Ip6::Address *SrpServer::GetSmallestOmrAddress() const
{
    const Ip6::Address *            smallestOmrAddress = nullptr;
    NetworkData::Iterator           iterator           = NetworkData::kIteratorInit;
    NetworkData::OnMeshPrefixConfig onMeshPrefixConfig;

    while (Get<NetworkData::Leader>().GetNextOnMeshPrefix(iterator, onMeshPrefixConfig) == OT_ERROR_NONE)
    {
        if (!Get<BorderRouter::RoutingManager>().IsValidOmrPrefixConfig(onMeshPrefixConfig))
        {
            continue;
        }

        for (auto addr = Get<ThreadNetif>().GetUnicastAddresses(); addr != nullptr; addr = addr->GetNext())
        {
            if (addr->GetAddress().MatchesPrefix(onMeshPrefixConfig.GetPrefix()))
            {
                if (smallestOmrAddress == nullptr || addr->GetAddress() < *smallestOmrAddress)
                {
                    smallestOmrAddress = &addr->GetAddress();
                }
            }
        }
    }

    return smallestOmrAddress;
}

otError SrpServer::PublishService()
{
    otError       error;
    const uint8_t serviceData[] = {kThreadServiceTypeSrpServer};
    uint8_t       serverData[sizeof(Ip6::Address) + sizeof(uint16_t)];

    OT_ASSERT(mSocket.IsBound());
    OT_ASSERT(!mPublishedAddress.IsUnspecified());

    memcpy(serverData, mPublishedAddress.GetBytes(), sizeof(mPublishedAddress));
    *reinterpret_cast<uint16_t *>(serverData + sizeof(mPublishedAddress)) =
        Encoding::BigEndian::HostSwap16(mSocket.GetSockName().mPort);

    SuccessOrExit(error =
                      Get<NetworkData::Local>().AddService(kThreadEnterpriseNumber, serviceData, sizeof(serviceData),
                                                           /* aServerStable */ true, serverData, sizeof(serverData)));
    Get<NetworkData::Notifier>().HandleServerDataUpdated();

exit:
    return error;
}

void SrpServer::UnpublishService()
{
    otError       error;
    const uint8_t serviceData[] = {kThreadServiceTypeSrpServer};

    OT_ASSERT(!mPublishedAddress.IsUnspecified());

    SuccessOrExit(
        error = Get<NetworkData::Local>().RemoveService(kThreadEnterpriseNumber, serviceData, sizeof(serviceData)));
    Get<NetworkData::Notifier>().HandleServerDataUpdated();

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnBr("failed to unpublish SRP service: %s", otThreadErrorToString(error));
    }
}

void SrpServer::HandleDnsUpdate(Message &               aMessage,
                                const Ip6::MessageInfo &aMessageInfo,
                                const Dns::Header &     aDnsHeader,
                                uint16_t                aOffset)
{
    otError   error = OT_ERROR_NONE;
    Dns::Zone zone;

    uint16_t headerOffset = aOffset - sizeof(aDnsHeader);

    otLogInfoBr("SRP server received DNS update from %s", aMessageInfo.GetPeerAddr().ToString().AsCString());

    SuccessOrExit(error = ProcessZoneSection(aMessage, aDnsHeader, aOffset, zone));

    if (mOutstandingUpdates.FindMatching(aDnsHeader.GetMessageId()) != nullptr)
    {
        otLogInfoBr("SRP server: drops duplicated SRP update request: messageId=%hu", aDnsHeader.GetMessageId());

        // Silently drop duplicate requests.
        // This could rarely happen, becuase the outstanding SRP update timer should
        // be much shorter than the SRP update retransmission timer.
        ExitNow(error = OT_ERROR_NONE);
    }

    // Per 2.3.2 of SRP draft 6, no prerequisites should be included in a SRP update.
    VerifyOrExit(aDnsHeader.GetPrerequisiteCount() == 0, error = OT_ERROR_FAILED);

    SuccessOrExit(error = ProcessUpdateSection(aMessage, aMessageInfo, aDnsHeader, zone, headerOffset, aOffset));

exit:
    if (error != OT_ERROR_NONE)
    {
        SendResponse(aDnsHeader, ErrorToResponseCode(error), aMessageInfo);
    }
}

otError SrpServer::ProcessZoneSection(const Message &    aMessage,
                                      const Dns::Header &aDnsHeader,
                                      uint16_t &         aOffset,
                                      Dns::Zone &        aZone)
{
    otError   error = OT_ERROR_NONE;
    Dns::Zone zone;

    VerifyOrExit(aDnsHeader.GetZoneCount() == 1, error = OT_ERROR_FAILED);

    SuccessOrExit(Dns::Name::ParseName(aMessage, aOffset));
    SuccessOrExit(aMessage.Read(aOffset, zone));
    aOffset += sizeof(zone);
    VerifyOrExit(zone.GetType() == Dns::ResourceRecord::kTypeSoa, error = OT_ERROR_FAILED);

    aZone = zone;

exit:
    return error;
}

otError SrpServer::ProcessUpdateSection(const Message &         aMessage,
                                        const Ip6::MessageInfo &aMessageInfo,
                                        const Dns::Header &     aDnsHeader,
                                        const Dns::Zone &       aZone,
                                        uint16_t                aHeaderOffset,
                                        uint16_t &              aOffset)
{
    otError error = OT_ERROR_NONE;
    Host *  host  = nullptr;
    Key *   key   = nullptr;

    host = Host::New();
    VerifyOrExit(host != nullptr, error = OT_ERROR_NO_BUFS);

    for (uint16_t i = 0; i < aDnsHeader.GetUpdateCount(); ++i)
    {
        char                name[Dns::Name::kMaxLength + 1];
        uint16_t            nameLength = sizeof(name);
        Dns::ResourceRecord update;

        SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, name, nameLength));
        SuccessOrExit(error = aMessage.Read(aOffset, update));
        aOffset += sizeof(update);

        if (update.GetClass() == Dns::ResourceRecord::kClassAny)
        {
            // Delete All RRsets from A Name
            VerifyOrExit(update.GetType() == Dns::ResourceRecord::kTypeAny && update.GetTtl() == 0 &&
                             update.GetLength() == 0,
                         error = OT_ERROR_FAILED);

            // TODO:
            continue;
        }

        VerifyOrExit(update.GetClass() == aZone.GetClass(), error = OT_ERROR_FAILED);

        if (update.GetType() == Dns::ResourceRecord::kTypePtr)
        {
            char     serviceName[Dns::Name::kMaxLength + 1];
            uint16_t serviceNameLength = sizeof(serviceName);

            SuccessOrExit(error =
                              Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, serviceName, serviceNameLength));
            VerifyOrExit(host->AddService(serviceName, serviceNameLength) != nullptr, error = OT_ERROR_NO_BUFS);
        }
        else if (update.GetType() == Dns::ResourceRecord::kTypeSrv)
        {
            Dns::SrvRecord srvRecord;
            Service *      service;
            char           hostName[Dns::Name::kMaxLength + 1];
            uint16_t       hostNameLength = sizeof(hostName);

            SuccessOrExit(error = aMessage.Read(aOffset - sizeof(update), srvRecord));
            aOffset += sizeof(srvRecord) - sizeof(update);

            SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, hostName, hostNameLength));
            VerifyOrExit(host->Matches(hostName), error = OT_ERROR_FAILED);

            service = host->AddService(name, nameLength);
            VerifyOrExit(service != nullptr, error = OT_ERROR_NO_BUFS);

            // We don't even need the priority and weight field.
            service->SetPort(srvRecord.GetPort());
        }
        else if (update.GetType() == Dns::ResourceRecord::kTypeTxt)
        {
            Service *service;

            service = host->AddService(name, nameLength);
            VerifyOrExit(service != nullptr, error = OT_ERROR_NO_BUFS);
            SuccessOrExit(error = service->SetTxtDataFromMessage(aMessage, aOffset, update.GetLength()));

            aOffset += update.GetLength();
        }
        else if (update.GetType() == Dns::ResourceRecord::kTypeAaaa)
        {
            Ip6::Address ip6Address;

            if (host->GetFullName() == nullptr)
            {
                SuccessOrExit(error = host->SetFullName(name, nameLength));
            }
            else
            {
                VerifyOrExit(host->Matches(name), error = OT_ERROR_FAILED);
            }
            VerifyOrExit(update.GetLength() == sizeof(ip6Address), error = OT_ERROR_PARSE);
            SuccessOrExit(error = aMessage.Read(aOffset, ip6Address));
            aOffset += sizeof(ip6Address);

            if (ip6Address.IsMulticast() || ip6Address.IsUnspecified() || ip6Address.IsLoopback() ||
                ip6Address.IsLinkLocal())
            {
                // We don't like those address because they connot be used for communication
                // with exterior devices.
                continue;
            }

            SuccessOrExit(error = host->AddIp6Address(ip6Address));
        }
        else if (update.GetType() == Dns::ResourceRecord::kTypeKey)
        {
            VerifyOrExit(key == nullptr, error = OT_ERROR_FAILED);

            key = Key::New();
            VerifyOrExit(key != nullptr, error = OT_ERROR_NO_BUFS);

            SuccessOrExit(error = key->SetKeyFromMessage(aMessage, aOffset, update.GetLength()));
            aOffset += update.GetLength();
        }
        else if (update.GetType() == Dns::ResourceRecord::kTypeA)
        {
            // Ignore IPv4 addresses.
            aOffset += update.GetLength();
        }
        else
        {
            ExitNow(error = OT_ERROR_FAILED);
        }
    }

    VerifyOrExit(host->GetFullName() != nullptr, error = OT_ERROR_FAILED);
    {
        uint8_t hostAddressesNum;
        host->GetAddresses(hostAddressesNum);
        VerifyOrExit(hostAddressesNum > 0, error = OT_ERROR_FAILED);
    }

    // Parse lease time and validate signature.
    SuccessOrExit(error = ProcessAdditionalSection(aMessage, aDnsHeader, aHeaderOffset, aOffset, host));

    HandleUpdate(aDnsHeader, host, aMessageInfo);

exit:
    if (error != OT_ERROR_NONE)
    {
        Host::Destroy(host);
        Key::Destroy(key);
    }

    return error;
}

otError SrpServer::ProcessAdditionalSection(const Message &    aMessage,
                                            const Dns::Header &aDnsHeader,
                                            uint16_t           aHeaderOffset,
                                            uint16_t &         aOffset,
                                            Host *             aHost)
{
    otError error        = OT_ERROR_NONE;
    bool    hasLease     = false;
    bool    hasSignature = false;

    // EDNS(0) and SIG(0) are expected.
    VerifyOrExit(aDnsHeader.GetAdditionalRecordsCount() == 2, error = OT_ERROR_FAILED);

    for (uint8_t i = 0; i < aDnsHeader.GetAdditionalRecordsCount(); ++i)
    {
        char                name[2]; // The root domain name (".") is expected.
        uint16_t            nameLength = sizeof(name);
        Dns::ResourceRecord record;

        SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, name, nameLength));
        SuccessOrExit(error = aMessage.Read(aOffset, record));

        if (record.GetType() == Dns::ResourceRecord::kTypeOpt)
        {
            Dns::UpdateLeaseOptRecord leaseRecord;

            SuccessOrExit(error = aMessage.Read(aOffset, leaseRecord));
            VerifyOrExit(leaseRecord.IsValid(), error = OT_ERROR_PARSE);

            aOffset += sizeof(Dns::ResourceRecord) + leaseRecord.GetLength();

            aHost->SetLease(leaseRecord.GetLeaseInterval());
            aHost->SetKeyLease(leaseRecord.GetKeyLeaseInterval());

            hasLease = true;
        }
        else if (record.GetType() == Dns::ResourceRecord::kTypeSig)
        {
            Dns::SigRecord sigRecord;
            uint16_t       rdataOffset = aOffset;
            char           signerName[Dns::Name::kMaxLength + 1];
            uint16_t       signerNameLength = sizeof(signerName);
            uint8_t        signature[Dns::SigRecord::kMaxSignatureLength];
            uint16_t       signatureLength;

            SuccessOrExit(error = aMessage.Read(aOffset, sigRecord));
            VerifyOrExit(sigRecord.IsValid(), error = OT_ERROR_PARSE);

            aOffset += sizeof(sigRecord);

            SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, signerName, signerNameLength));

            signatureLength = sigRecord.GetLength() - (aOffset - rdataOffset);
            VerifyOrExit(signatureLength <= sizeof(signature), error = OT_ERROR_NO_BUFS);
            VerifyOrExit(aMessage.ReadBytes(aOffset, signature, signatureLength) == signatureLength,
                         error = OT_ERROR_PARSE);
            aOffset += signatureLength;

            hasSignature = true;

            // TODO: signature validation.
        }
        else
        {
            ExitNow(error = OT_ERROR_FAILED);
        }
    }

    VerifyOrExit(hasLease && hasSignature, error = OT_ERROR_FAILED);

exit:
    return error;
}

void SrpServer::HandleUpdate(const Dns::Header &aDnsHeader, Host *aHost, const Ip6::MessageInfo &aMessageInfo)
{
    otError error  = OT_ERROR_NONE;
    int     events = 0;
    bool    hasNameConflicts;

    hasNameConflicts = HasNameConflictsWith(aHost);
    VerifyOrExit(!hasNameConflicts);

    if (aHost->GetLease() == 0)
    {
        events |= OT_SRP_SERVER_EVENT_UNREGISTER_HOST;

        // We always unregister all services when unregistering the host.
        events |= OT_SRP_SERVER_EVENT_UNREGISTER_SERVICE;

        aHost->RemoveAllServices();
    }
    else
    {
        events |= OT_SRP_SERVER_EVENT_REGISTER_HOST;

        if (aHost->GetServices() != nullptr)
        {
            events |= OT_SRP_SERVER_EVENT_REGISTER_SERVICE;
        }
    }

    if (mServiceHandler != nullptr)
    {
        UpdateMetadata *update = UpdateMetadata::New(aDnsHeader, aHost, aMessageInfo);

        SuccessOrExit(error = mOutstandingUpdates.Add(*update));
        mOutstandingUpdatesTimer.StartAt(mOutstandingUpdates.GetHead()->GetExpireTime(), 0);

        mServiceHandler(aHost, events, kDefaultEventsHandlerTimeout, update);
    }
    else
    {
        HandleServiceEventResult(OT_ERROR_NONE, aDnsHeader, *aHost, aMessageInfo);
    }

exit:
    if (hasNameConflicts)
    {
        SendResponse(aDnsHeader, Dns::Header::kResponseYxDomain, aMessageInfo);
    }
    else if (error != OT_ERROR_NONE)
    {
        SendResponse(aDnsHeader, ErrorToResponseCode(error), aMessageInfo);
    }
}

void SrpServer::SendResponse(const Dns::Header &     aHeader,
                             Dns::Header::Response   aResponseCode,
                             const Ip6::MessageInfo &aMessageInfo)
{
    otError     error;
    Message *   response = nullptr;
    Dns::Header header;

    header.SetMessageId(aHeader.GetMessageId());
    header.SetType(Dns::Header::kTypeResponse);
    header.SetQueryType(aHeader.GetQueryType());
    header.SetResponseCode(aResponseCode);

    response = Get<Ip6::Udp>().NewMessage(0);
    VerifyOrExit(response != nullptr, error = OT_ERROR_NO_BUFS);
    SuccessOrExit(error = response->Append(header));
    SuccessOrExit(error = mSocket.SendTo(*response, aMessageInfo));

    if (aResponseCode != Dns::Header::kResponseSuccess)
    {
        otLogInfoBr("SRP server sent fail response: %d", aResponseCode);
    }
    else
    {
        otLogInfoBr("SRP server sent success response");
    }

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnBr("SRP server failed to send response: %s", otThreadErrorToString(error));
        FreeMessageOnError(response, error);
    }
}

void SrpServer::SendResponse(const Dns::Header &     aHeader,
                             uint32_t                aLease,
                             uint32_t                aKeyLease,
                             const Ip6::MessageInfo &aMessageInfo)
{
    otError                   error;
    Message *                 response = nullptr;
    Dns::Header               header;
    char                      kRootName[2] = ".";
    Dns::UpdateLeaseOptRecord leaseRecord;

    header.SetMessageId(aHeader.GetMessageId());
    header.SetType(Dns::Header::kTypeResponse);
    header.SetQueryType(aHeader.GetQueryType());
    header.SetResponseCode(Dns::Header::kResponseSuccess);
    header.SetAdditionalRecordsCount(1);

    leaseRecord.Init();
    leaseRecord.SetLeaseInterval(aLease);
    leaseRecord.SetKeyLeaseInterval(aKeyLease);

    response = Get<Ip6::Udp>().NewMessage(0);
    VerifyOrExit(response != nullptr, error = OT_ERROR_NO_BUFS);
    SuccessOrExit(error = response->Append(header));
    SuccessOrExit(error = Dns::Name::AppendName(kRootName, *response));
    SuccessOrExit(error = response->Append(leaseRecord));

    SuccessOrExit(error = mSocket.SendTo(*response, aMessageInfo));

    otLogInfoBr("SRP server: sent response with granted lease: %u and key lease: %u", aLease, aKeyLease);

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnBr("SRP server failed to send response: %s", otThreadErrorToString(error));
        FreeMessageOnError(response, error);
    }
}

void SrpServer::HandleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    static_cast<SrpServer *>(aContext)->HandleUdpReceive(*static_cast<Message *>(aMessage),
                                                         *static_cast<const Ip6::MessageInfo *>(aMessageInfo));
}

void SrpServer::HandleUdpReceive(Message &aMessage, const Ip6::MessageInfo &aMessageInfo)
{
    otError     error;
    Dns::Header dnsHeader;
    uint16_t    offset = aMessage.GetOffset();

    SuccessOrExit(error = aMessage.Read(offset, dnsHeader));
    offset += sizeof(dnsHeader);

    // Handles only queries.
    VerifyOrExit(dnsHeader.GetType() == Dns::Header::Type::kTypeQuery, error = OT_ERROR_DROP);

    switch (dnsHeader.GetQueryType())
    {
    case Dns::Header::kQueryTypeUpdate:
        HandleDnsUpdate(aMessage, aMessageInfo, dnsHeader, offset);
        break;
    case Dns::Header::kQueryTypeStandard:
        // TODO:
        break;
    default:
        error = OT_ERROR_DROP;
        break;
    }

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogInfoBr("SRP server failed to handle DNS message: %s", otThreadErrorToString(error));
    }
}

void SrpServer::HandleLeaseTimer(Timer &aTimer)
{
    aTimer.GetOwner<SrpServer>().HandleLeaseTimer();
}

void SrpServer::HandleLeaseTimer(void)
{
    TimeMilli now                = TimerMilli::GetNow();
    TimeMilli earliestExpireTime = now.GetDistantFuture();
    auto      host               = mHosts.GetHead();

    while (host != nullptr)
    {
        auto nextHost = host->GetNext();

        if (host->GetKeyExpireTime() <= now)
        {
            otLogInfoBr("SRP server: KEY LEASE of host %s expires", host->GetFullName());

            // Removes the whole host and all services if the KEY RR expires.
            RemoveHost(host);
        }
        else if (host->IsDeleted())
        {
            // The host has been deleted, but the hostname & service instance names retains.

            auto service = host->GetServices();

            earliestExpireTime = OT_MIN(earliestExpireTime, host->GetKeyExpireTime());

            // Check if any service instance name expires.
            while (service != nullptr)
            {
                OT_ASSERT(service->IsDeleted());

                auto nextService = service->GetNext();

                if (service->GetKeyExpireTime() <= now)
                {
                    otLogInfoBr("SRP server: KEY LEASE of service %s expires", service->GetFullName());
                    host->RemoveService(service);
                }
                else
                {
                    earliestExpireTime = OT_MIN(earliestExpireTime, service->GetKeyExpireTime());
                }

                service = nextService;
            }
        }
        else if (host->GetExpireTime() <= now)
        {
            otLogInfoBr("SRP server: LEASE of host %s expires", host->GetFullName());

            // If the host expires, delete all resources of this host and its services.
            host->ClearResources();

            earliestExpireTime = OT_MIN(earliestExpireTime, host->GetKeyExpireTime());
        }
        else
        {
            // The host doesn't expire, check if any service expires or is explicitly removed.

            OT_ASSERT(!host->IsDeleted());

            auto service = host->GetServices();

            earliestExpireTime = OT_MIN(earliestExpireTime, host->GetExpireTime());

            while (service != nullptr)
            {
                auto nextService = service->GetNext();

                if (service->IsDeleted())
                {
                    // The service has been deleted but the name retains.
                    earliestExpireTime = OT_MIN(earliestExpireTime, service->GetKeyExpireTime());
                }
                else if (service->GetExpireTime() <= now)
                {
                    otLogInfoBr("SRP server: LEASE of service %s expires", service->GetFullName());

                    // The service gets expired, delete it.

                    service->ClearResources();

                    earliestExpireTime = OT_MIN(earliestExpireTime, service->GetKeyExpireTime());
                }
                else
                {
                    earliestExpireTime = OT_MIN(earliestExpireTime, service->GetExpireTime());
                }

                service = nextService;
            }
        }

        host = nextHost;
    }

    if (earliestExpireTime != now.GetDistantFuture())
    {
        if (!mLeaseTimer.IsRunning() || earliestExpireTime <= mLeaseTimer.GetFireTime())
        {
            otLogInfoBr("SRP server: lease timer is scheduled in %u seconds", (earliestExpireTime - now) / 1000);
            mLeaseTimer.StartAt(earliestExpireTime, 0);
        }
    }
    else
    {
        otLogInfoBr("SRP server: lease timer is stopped");
        mLeaseTimer.Stop();
    }
}

void SrpServer::HandleOutstandingUpdatesTimer(Timer &aTimer)
{
    aTimer.GetOwner<SrpServer>().HandleOutstandingUpdatesTimer();
}

void SrpServer::HandleOutstandingUpdatesTimer(void)
{
    while (!mOutstandingUpdates.IsEmpty() && mOutstandingUpdates.GetTail()->GetExpireTime() <= TimerMilli::GetNow())
    {
        HandleServiceEventResult(OT_ERROR_RESPONSE_TIMEOUT, mOutstandingUpdates.GetTail());
    }
}

SrpServer::Service *SrpServer::Service::New(const char *aName, uint8_t aNameLength)
{
    void *   buf;
    Service *service = nullptr;

    buf = Instance::Get().HeapCAlloc(1, sizeof(Service));
    VerifyOrExit(buf != nullptr);

    service = new (buf) Service();
    if (service->SetFullName(aName, aNameLength) != OT_ERROR_NONE)
    {
        Destroy(service);
        service = nullptr;
    }

exit:
    return service;
}

void SrpServer::Service::Destroy(Service *aService)
{
    if (aService != nullptr)
    {
        aService->~Service();
        Instance::Get().HeapFree(aService);
    }
}

SrpServer::Service::Service()
{
    mFullName  = nullptr;
    mPort      = 0;
    mHost      = nullptr;
    mTxtData   = nullptr;
    mTxtLength = 0;
    mDeleted   = false;
    mNext      = nullptr;
}

SrpServer::Service::~Service()
{
    Instance::Get().HeapFree(mFullName);
    Instance::Get().HeapFree(mTxtData);
}

otError SrpServer::Service::SetFullName(const char *aName, uint8_t aLength)
{
    OT_ASSERT(aName != nullptr);

    otError error    = OT_ERROR_NONE;
    char *  nameCopy = static_cast<char *>(Instance::Get().HeapCAlloc(1, aLength + 1));

    VerifyOrExit(nameCopy != nullptr, error = OT_ERROR_NO_BUFS);
    strncpy(nameCopy, aName, aLength);
    nameCopy[aLength] = '\0';

    Instance::Get().HeapFree(mFullName);
    mFullName = nameCopy;

    mTimeLastUpdate = TimerMilli::GetNow();

exit:
    return error;
}

TimeMilli SrpServer::Service::GetExpireTime(void) const
{
    OT_ASSERT(!IsDeleted());
    OT_ASSERT(!GetHost().IsDeleted());

    return mTimeLastUpdate + GetHost().GetLease() * 1000;
}

TimeMilli SrpServer::Service::GetKeyExpireTime(void) const
{
    return mTimeLastUpdate + GetHost().GetKeyLease() * 1000;
}

otError SrpServer::Service::SetTxtData(const uint8_t *aTxtData, uint16_t aTxtDataLength)
{
    otError  error = OT_ERROR_NONE;
    uint8_t *txtData;

    txtData = static_cast<uint8_t *>(Instance::Get().HeapCAlloc(1, aTxtDataLength));
    VerifyOrExit(txtData != nullptr, error = OT_ERROR_NO_BUFS);

    memcpy(txtData, aTxtData, aTxtDataLength);

    Instance::Get().HeapFree(mTxtData);
    mTxtData   = txtData;
    mTxtLength = aTxtDataLength;

    mTimeLastUpdate = TimerMilli::GetNow();

exit:
    return error;
}

otError SrpServer::Service::SetTxtDataFromMessage(const Message &aMessage, uint16_t aOffset, uint16_t aLength)
{
    otError  error = OT_ERROR_NONE;
    uint8_t *txtData;

    txtData = static_cast<uint8_t *>(Instance::Get().HeapCAlloc(1, aLength));
    VerifyOrExit(txtData != nullptr, error = OT_ERROR_NO_BUFS);
    VerifyOrExit(aMessage.ReadBytes(aOffset, txtData, aLength) == aLength, error = OT_ERROR_PARSE);

    Instance::Get().HeapFree(mTxtData);
    mTxtData   = txtData;
    mTxtLength = aLength;

    mTimeLastUpdate = TimerMilli::GetNow();

exit:
    if (error != OT_ERROR_NONE)
    {
        Instance::Get().HeapFree(txtData);
    }

    return error;
}

void SrpServer::Service::ClearResources(void)
{
    mPort = 0;
    Instance::Get().HeapFree(mTxtData);
    mTxtData   = nullptr;
    mTxtLength = 0;

    mTimeLastUpdate = TimerMilli::GetNow();
}

otError SrpServer::Service::CopyResourcesFrom(const Service &aService)
{
    otError error;

    SuccessOrExit(error = SetTxtData(aService.mTxtData, aService.mTxtLength));
    mPort = aService.mPort;

    mTimeLastUpdate = TimerMilli::GetNow();

exit:
    return error;
}

bool SrpServer::Service::Matches(const char *aName) const
{
    return (mFullName != nullptr) && (strcmp(mFullName, aName) == 0);
}

bool SrpServer::Service::operator==(const Service &aOther) const
{
    return strcmp(mFullName, aOther.mFullName) == 0 && mPort == aOther.mPort && mTxtLength == aOther.mTxtLength &&
           memcmp(mTxtData, aOther.mTxtData, mTxtLength) == 0;
}

SrpServer::Host *SrpServer::Host::New()
{
    void *buf;
    Host *host = nullptr;

    buf = Instance::Get().HeapCAlloc(1, sizeof(Host));
    VerifyOrExit(buf != nullptr);

    host = new (buf) Host();

exit:
    return host;
}

void SrpServer::Host::Destroy(Host *aHost)
{
    if (aHost != nullptr)
    {
        aHost->~Host();
        Instance::Get().HeapFree(aHost);
    }
}

SrpServer::Host::Host()
    : mKey(nullptr)
{
    mFullName     = nullptr;
    mAddressesNum = 0;
    mNext         = nullptr;
    mKey          = nullptr;
    mLease        = 0;
    mKeyLease     = 0;
}

SrpServer::Host::~Host()
{
    RemoveAllServices();
    Instance::Get().HeapFree(mFullName);
    Instance::Get().HeapFree(mKey);
}

otError SrpServer::Host::SetFullName(const char *aName, uint8_t aLength)
{
    OT_ASSERT(aName != nullptr);

    otError error    = OT_ERROR_NONE;
    char *  nameCopy = static_cast<char *>(Instance::Get().HeapCAlloc(1, aLength + 1));

    VerifyOrExit(nameCopy != nullptr, error = OT_ERROR_NO_BUFS);
    strcpy(nameCopy, aName);

    if (mFullName != nullptr)
    {
        Instance::Get().HeapFree(mFullName);
    }
    mFullName = nameCopy;

    mTimeLastUpdate = TimerMilli::GetNow();

exit:
    return error;
}

void SrpServer::Host::SetLease(uint32_t aLease)
{
    mLease          = aLease;
    mTimeLastUpdate = TimerMilli::GetNow();
}

void SrpServer::Host::SetKeyLease(uint32_t aKeyLease)
{
    mKeyLease       = aKeyLease;
    mTimeLastUpdate = TimerMilli::GetNow();
}

TimeMilli SrpServer::Host::GetExpireTime(void) const
{
    OT_ASSERT(!IsDeleted());

    return mTimeLastUpdate + mLease * 1000;
}

TimeMilli SrpServer::Host::GetKeyExpireTime(void) const
{
    return mTimeLastUpdate + mKeyLease * 1000;
}

SrpServer::Service *SrpServer::Host::AddService(const char *aName, uint16_t aNameLength)
{
    Service *service;

    service = FindService(aName);
    if (service == nullptr)
    {
        service = Service::New(aName, aNameLength);
        if (service != nullptr)
        {
            AddService(service);
        }
    }

    return service;
}

void SrpServer::Host::AddService(SrpServer::Service *aService)
{
    IgnoreError(mServices.Add(*aService));
    aService->mHost = this;
}

void SrpServer::Host::RemoveService(Service *aService)
{
    mServices.Remove(*aService);
    Service::Destroy(aService);
}

void SrpServer::Host::RemoveAllServices(void)
{
    while (!mServices.IsEmpty())
    {
        RemoveService(mServices.GetHead());
    }
}

void SrpServer::Host::ClearResources(void)
{
    otLogInfoBr("SRP server: removes all resources of host %s, keep its name", GetFullName());

    mAddressesNum = 0;
    mLease        = 0;

    // We must removes all service when removing a host.
    for (auto service = GetServices(); service != nullptr; service = service->GetNext())
    {
        service->ClearResources();
    }

    mTimeLastUpdate = TimerMilli::GetNow();
}

void SrpServer::Host::CopyResourcesFrom(const Host &aHost)
{
    memcpy(mAddresses, aHost.mAddresses, aHost.mAddressesNum * sizeof(mAddresses[0]));
    mAddressesNum = aHost.mAddressesNum;
    mLease        = aHost.mLease;
    mKeyLease     = aHost.mKeyLease;

    mTimeLastUpdate = TimerMilli::GetNow();
}

SrpServer::Service *SrpServer::Host::FindService(const char *aFullName)
{
    return mServices.FindMatching(aFullName);
}

otError SrpServer::Host::AddIp6Address(const Ip6::Address &aIp6Address)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(mAddressesNum < kMaxAddressesNum, error = OT_ERROR_NO_BUFS);

    mAddresses[mAddressesNum++] = aIp6Address;
    mTimeLastUpdate             = TimerMilli::GetNow();

exit:
    return error;
}

bool SrpServer::Host::Matches(const char *aName) const
{
    return mFullName != nullptr && strcmp(mFullName, aName) == 0;
}

bool SrpServer::Host::operator==(const Host &aOther) const
{
    return strcmp(mFullName, aOther.mFullName) == 0 && mAddressesNum == aOther.mAddressesNum &&
           memcmp(mAddresses, aOther.mAddresses, sizeof(mAddresses[0]) * mAddressesNum) == 0 && *mKey == *aOther.mKey;
}

SrpServer::Key *SrpServer::Key::New()
{
    void *buf;
    Key * key = nullptr;

    buf = Instance::Get().HeapCAlloc(1, sizeof(Key));
    VerifyOrExit(buf != nullptr);

    key = new (buf) Key();

exit:
    return key;
}

void SrpServer::Key::Destroy(Key *aKey)
{
    if (aKey != nullptr)
    {
        aKey->~Key();
        Instance::Get().HeapFree(aKey);
    }
}

SrpServer::Key::Key()
    : mFlags(0)
    , mProtocol(0)
    , mAlgorithm(0)
    , mKey(nullptr)
    , mKeyLength(0)
{
}

SrpServer::Key::~Key()
{
    Instance::Get().HeapFree(mKey);
}

otError SrpServer::Key::SetKeyFromMessage(const Message &aMessage, uint16_t aOffset, uint16_t aLength)
{
    otError error = OT_ERROR_NONE;

    OT_TOOL_PACKED_BEGIN
    struct Stub
    {
        uint16_t flags;
        uint8_t  protocol;
        uint8_t  algorithm;
    } OT_TOOL_PACKED_END stub;

    uint8_t *key       = nullptr;
    uint16_t keyLength = aLength - sizeof(Stub);

    SuccessOrExit(error = aMessage.Read(aOffset, stub));
    aOffset += sizeof(stub);

    key = static_cast<uint8_t *>(Instance::Get().HeapCAlloc(1, keyLength));
    VerifyOrExit(key != nullptr, error = OT_ERROR_NO_BUFS);

    VerifyOrExit(aMessage.ReadBytes(aOffset, key, keyLength) == keyLength, error = OT_ERROR_PARSE);

    mFlags     = stub.flags;
    mProtocol  = stub.protocol;
    mAlgorithm = stub.algorithm;
    Instance::Get().HeapFree(mKey);
    mKey       = key;
    mKeyLength = keyLength;

exit:
    if (error != OT_ERROR_NONE)
    {
        Instance::Get().HeapFree(key);
    }

    return error;
}

bool SrpServer::Key::operator==(const Key &aOther) const
{
    return mFlags == aOther.mFlags && mProtocol == aOther.mProtocol && mAlgorithm == aOther.mAlgorithm &&
           mKeyLength == aOther.mKeyLength && memcmp(mKey, aOther.mKey, mKeyLength) == 0;
}

SrpServer::UpdateMetadata *SrpServer::UpdateMetadata::New(const Dns::Header &     aHeader,
                                                          Host *                  aHost,
                                                          const Ip6::MessageInfo &aMessageInfo)
{
    void *          buf;
    UpdateMetadata *update = nullptr;

    buf = Instance::Get().HeapCAlloc(1, sizeof(UpdateMetadata));
    VerifyOrExit(buf != nullptr);

    update = new (buf) UpdateMetadata(aHeader, aHost, aMessageInfo);

exit:
    return update;
}

void SrpServer::UpdateMetadata::Destroy(UpdateMetadata *aUpdateMetadata)
{
    if (aUpdateMetadata != nullptr)
    {
        aUpdateMetadata->~UpdateMetadata();
        Instance::Get().HeapFree(aUpdateMetadata);
    }
}

SrpServer::UpdateMetadata::UpdateMetadata(const Dns::Header &aHeader, Host *aHost, const Ip6::MessageInfo &aMessageInfo)
    : mExpireTime(TimerMilli::GetNow() + kDefaultEventsHandlerTimeout)
    , mDnsHeader(aHeader)
    , mHost(aHost)
    , mMessageInfo(aMessageInfo)
    , mNext(nullptr)
{
}

} // namespace BorderRouter

} // namespace ot

#endif // OPENTHREAD_CONFIG_SRP_SERVER_ENABLE
