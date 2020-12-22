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

namespace Srp {

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
    , mAdvertisingHandler(nullptr)
    , mAdvertisingHandlerContext(nullptr)
    , mMinLease(kDefaultMinLease)
    , mMaxLease(kDefaultMaxLease)
    , mMinKeyLease(kDefaultMinKeyLease)
    , mMaxKeyLease(kDefaultMaxKeyLease)
    , mLeaseTimer(aInstance, HandleLeaseTimer, this)
    , mOutstandingUpdatesTimer(aInstance, HandleOutstandingUpdatesTimer, this)
{
}

void SrpServer::SetServiceHandler(otSrpServerAdvertisingHandler aServiceHandler, void *aServiceHandlerContext)
{
    mAdvertisingHandler        = aServiceHandler;
    mAdvertisingHandlerContext = aServiceHandlerContext;
}

bool SrpServer::IsRunning(void) const
{
    return mSocket.IsBound();
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
    otLogInfoSrp("SRP server: permanently removes host %s", aHost->GetFullName());
    IgnoreError(mHosts.Remove(*aHost));
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

void SrpServer::HandleAdvertisingResult(const Host *aHost, otError aError)
{
    UpdateMetadata *update = mOutstandingUpdates.FindMatching(aHost);

    if (update != nullptr)
    {
        HandleAdvertisingResult(update, aError);
    }
}

void SrpServer::HandleAdvertisingResult(UpdateMetadata *aUpdate, otError aError)
{
    HandleServiceEventResult(aError, aUpdate->GetDnsHeader(), aUpdate->GetHost(), aUpdate->GetMessageInfo());

    IgnoreError(mOutstandingUpdates.Remove(*aUpdate));
    UpdateMetadata::Destroy(aUpdate);

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
        otLogInfoSrp("SRP server: removes host %s", aHost.GetFullName());

        if (aHost.GetKeyLease() == 0)
        {
            otLogInfoSrp("SRP server: removes key of host %s", aHost.GetFullName());

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

        otLogInfoSrp("SRP server: updates host %s", existingHost->GetFullName());

        existingHost->CopyResourcesFrom(aHost);

        for (auto service = aHost.GetServices(); service != nullptr; service = service->GetNext())
        {
            auto existingService = existingHost->AddService(service->GetFullName());

            VerifyOrExit(existingService != nullptr, aError = OT_ERROR_NO_BUFS);
            SuccessOrExit(aError = existingService->CopyResourcesFrom(*service));

            otLogInfoSrp("SRP server: adds/updates service %s", existingHost->GetFullName());
        }

        Host::Destroy(&aHost);
    }
    else
    {
        otLogInfoSrp("SRP server: adds new host %s", aHost.GetFullName());

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

    VerifyOrExit(!IsRunning(), error = OT_ERROR_ALREADY);

    SuccessOrExit(error = mSocket.Open(HandleUdpReceive, this));
    SuccessOrExit(error = mSocket.Bind(0));

    SuccessOrExit(error = PublishService());

    otLogInfoSrp("SRP server: starts listening on port %hu", mSocket.GetSockName().mPort);

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogCritSrp("failed to start SRP server: %s", otThreadErrorToString(error));
        // Cleanup any resources we may have allocated.
        Stop();
    }
}

void SrpServer::Stop()
{
    VerifyOrExit(IsRunning());

    UnpublishService();

    while (!mHosts.IsEmpty())
    {
        Host::Destroy(mHosts.Pop());
    }

    while (!mOutstandingUpdates.IsEmpty())
    {
        UpdateMetadata::Destroy(mOutstandingUpdates.Pop());
    }

    mLeaseTimer.Stop();
    mOutstandingUpdatesTimer.Stop();

    otLogInfoSrp("SRP server: stops listening on %hu", mSocket.GetSockName().mPort);
    IgnoreError(mSocket.Close());

exit:
    return;
}

void SrpServer::HandleNotifierEvents(Events aEvents)
{
    VerifyOrExit(mEnabled);

    if (aEvents.Contains(kEventThreadRoleChanged))
    {
        if (Get<Mle::MleRouter>().IsAttached())
        {
            Start();
        }
        else
        {
            Stop();
        }
    }

exit:
    return;
}

otError SrpServer::PublishService()
{
    otError       error;
    const uint8_t serviceData[] = {kThreadServiceTypeSrpServer};
    uint8_t       serverData[sizeof(uint16_t)];

    OT_ASSERT(mSocket.IsBound());

    *reinterpret_cast<uint16_t *>(serverData) = Encoding::BigEndian::HostSwap16(mSocket.GetSockName().mPort);

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

    SuccessOrExit(
        error = Get<NetworkData::Local>().RemoveService(kThreadEnterpriseNumber, serviceData, sizeof(serviceData)));
    Get<NetworkData::Notifier>().HandleServerDataUpdated();

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnSrp("failed to unpublish SRP service: %s", otThreadErrorToString(error));
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

    otLogInfoSrp("SRP server: received DNS update from %s", aMessageInfo.GetPeerAddr().ToString().AsCString());

    SuccessOrExit(error = ProcessZoneSection(aMessage, aDnsHeader, aOffset, zone));

    if (mOutstandingUpdates.FindMatching(aDnsHeader.GetMessageId()) != nullptr)
    {
        otLogInfoSrp("SRP server: drops duplicated SRP update request: messageId=%hu", aDnsHeader.GetMessageId());

        // Silently drop duplicate requests.
        // This could rarely happen, because the outstanding SRP update timer should
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

    host = Host::New();
    VerifyOrExit(host != nullptr, error = OT_ERROR_NO_BUFS);

    // Enumerate over all Service Discovery Instructions before processing any other records.
    // So that we will know whether a name is a hostname or service instance name when processing
    // a "Delete All RRsets from a name" record.
    SuccessOrExit(error = HandleDiscoveryInstructions(host, aMessage, aDnsHeader, aZone, aHeaderOffset, aOffset));

    for (uint16_t i = 0; i < aDnsHeader.GetUpdateCount(); ++i)
    {
        char                name[Dns::Name::kMaxLength + 1];
        Dns::ResourceRecord record;

        SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, name, sizeof(name)));
        SuccessOrExit(error = aMessage.Read(aOffset, record));

        if (record.GetClass() == Dns::ResourceRecord::kClassAny)
        {
            // Delete All RRsets from a name.
            VerifyOrExit(record.GetType() == Dns::ResourceRecord::kTypeAny && record.GetTtl() == 0 &&
                             record.GetLength() == 0,
                         error = OT_ERROR_FAILED);
            SuccessOrExit(error = HandleDeleteAllResources(host, name));

            aOffset += record.GetSize();
            continue;
        }

        VerifyOrExit(record.GetClass() == aZone.GetClass(), error = OT_ERROR_FAILED);

        if (record.GetType() == Dns::ResourceRecord::kTypePtr)
        {
            // Skip, as we have already processed all PTR RRs.
            aOffset += record.GetSize();
        }
        else if (record.GetType() == Dns::ResourceRecord::kTypeSrv)
        {
            Dns::SrvRecord srvRecord;
            Service *      service;
            char           hostName[Dns::Name::kMaxLength + 1];
            uint16_t       hostNameLength = sizeof(hostName);

            SuccessOrExit(error = aMessage.Read(aOffset, srvRecord));
            aOffset += sizeof(srvRecord);

            SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, hostName, hostNameLength));
            VerifyOrExit(host->Matches(hostName), error = OT_ERROR_FAILED);

            service = host->FindService(name);
            VerifyOrExit(service != nullptr && !service->IsDeleted(), error = OT_ERROR_FAILED);

            // Make sure that this is the first SRV RR for this service.
            VerifyOrExit(service->mPort == 0, error = OT_ERROR_FAILED);
            service->mPort = srvRecord.GetPort();
        }
        else if (record.GetType() == Dns::ResourceRecord::kTypeTxt)
        {
            Service *service = host->FindService(name);
            VerifyOrExit(service != nullptr && !service->IsDeleted(), error = OT_ERROR_FAILED);

            aOffset += sizeof(record);
            SuccessOrExit(error = service->SetTxtDataFromMessage(aMessage, aOffset, record.GetLength()));
            aOffset += record.GetLength();
        }
        else if (record.GetType() == Dns::ResourceRecord::kTypeAaaa)
        {
            Dns::AaaaRecord aaaaRecord;

            if (host->GetFullName() == nullptr)
            {
                SuccessOrExit(error = host->SetFullName(name));
            }
            else
            {
                VerifyOrExit(host->Matches(name), error = OT_ERROR_FAILED);
            }

            SuccessOrExit(error = aMessage.Read(aOffset, aaaaRecord));
            VerifyOrExit(aaaaRecord.IsValid(), error = OT_ERROR_PARSE);

            if (aaaaRecord.GetAddress().IsMulticast() || aaaaRecord.GetAddress().IsUnspecified() ||
                aaaaRecord.GetAddress().IsLoopback() || aaaaRecord.GetAddress().IsLinkLocal())
            {
                // We don't like those address because they cannot be used for communication
                // with exterior devices.
                continue;
            }

            SuccessOrExit(error = host->AddIp6Address(aaaaRecord.GetAddress()));

            aOffset += aaaaRecord.GetSize();
        }
        else if (record.GetType() == Dns::ResourceRecord::kTypeA)
        {
            // Ignore IPv4 addresses.
            aOffset += record.GetSize();
        }
        else if (record.GetType() == Dns::ResourceRecord::kTypeKey)
        {
            // We currently support only ECDSA P-256.
            Dns::Ecdsa256KeyRecord key;

            VerifyOrExit(host->GetKey() == nullptr, error = OT_ERROR_FAILED);

            SuccessOrExit(error = aMessage.Read(aOffset, key));
            VerifyOrExit(key.IsValid(), error = OT_ERROR_PARSE);

            host->SetKey(key);

            aOffset += record.GetSize();
        }
        else
        {
            ExitNow(error = OT_ERROR_FAILED);
        }
    }

    // Verify that we have a complete Host Description Instruction.

    VerifyOrExit(host->GetFullName() != nullptr, error = OT_ERROR_FAILED);
    VerifyOrExit(host->GetKey() != nullptr, error = OT_ERROR_FAILED);
    {
        uint8_t hostAddressesNum;

        host->GetAddresses(hostAddressesNum);

        // There MUST be at least one valid address.
        VerifyOrExit(hostAddressesNum > 0, error = OT_ERROR_FAILED);
    }

    // Parse lease time and validate signature.
    SuccessOrExit(error = ProcessAdditionalSection(aMessage, aDnsHeader, aHeaderOffset, aOffset, host));

    HandleUpdate(aDnsHeader, host, aMessageInfo);

exit:
    if (error != OT_ERROR_NONE)
    {
        Host::Destroy(host);
    }

    return error;
}

otError SrpServer::HandleDiscoveryInstructions(Host *             aHost,
                                               const Message &    aMessage,
                                               const Dns::Header &aDnsHeader,
                                               const Dns::Zone &  aZone,
                                               uint16_t           aHeaderOffset,
                                               uint16_t           aOffset)
{
    otError error;

    for (uint16_t i = 0; i < aDnsHeader.GetUpdateCount(); ++i)
    {
        char                name[Dns::Name::kMaxLength + 1];
        Dns::ResourceRecord record;
        char                serviceName[Dns::Name::kMaxLength + 1];
        Service *           service;

        SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, name, sizeof(name)));
        SuccessOrExit(error = aMessage.Read(aOffset, record));

        aOffset += sizeof(record);

        if (record.GetType() == Dns::ResourceRecord::kTypePtr)
        {
            SuccessOrExit(error =
                              Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, serviceName, sizeof(serviceName)));
        }
        else
        {
            aOffset += record.GetLength();
            continue;
        }

        VerifyOrExit(record.GetClass() == Dns::ResourceRecord::kClassNone || record.GetClass() == aZone.GetClass(),
                     error = OT_ERROR_FAILED);

        // TODO: check if the RR name and the full service name matches.

        service = aHost->FindService(serviceName);
        VerifyOrExit(service == nullptr, error = OT_ERROR_FAILED);
        service = aHost->AddService(serviceName);
        VerifyOrExit(service != nullptr, error = OT_ERROR_NO_BUFS);

        // This RR is a "Delete an RR from an RRset" update when the CLASS is NONE.
        service->SetDeleted(record.GetClass() == Dns::ResourceRecord::kClassNone);
    }

exit:
    return error;
}

otError SrpServer::HandleDeleteAllResources(Host *aHost, const char *aName)
{
    otError  error = OT_ERROR_NONE;
    Service *service;

    service = aHost->FindService(aName);
    if (service != nullptr)
    {
        service->ClearResources();
    }
    else
    {
        if (aHost->GetFullName())
        {
            VerifyOrExit(strcmp(aName, aHost->GetFullName()) == 0, error = OT_ERROR_FAILED);
        }
        else
        {
            SuccessOrExit(error = aHost->SetFullName(aName));
        }
        aHost->ClearResources();
    }

exit:
    return error;
}

otError SrpServer::ProcessAdditionalSection(const Message &    aMessage,
                                            const Dns::Header &aDnsHeader,
                                            uint16_t           aHeaderOffset,
                                            uint16_t &         aOffset,
                                            Host *             aHost)
{
    otError                   error = OT_ERROR_NONE;
    char                      name[2]; // The root domain name (".") is expected.
    Dns::UpdateLeaseOptRecord leaseRecord;

    Dns::SigRecord sigRecord;
    uint16_t       sigOffset;
    uint16_t       sigRdataOffset;
    char           signerName[Dns::Name::kMaxLength + 1];
    uint16_t       signerNameLength = sizeof(signerName);
    uint8_t        signature[Dns::SigRecord::kMaxSignatureLength];
    uint16_t       signatureLength;

    VerifyOrExit(aDnsHeader.GetAdditionalRecordsCount() == 2, error = OT_ERROR_FAILED);

    // EDNS(0) Update Lease Option.

    SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, name, sizeof(name)));
    SuccessOrExit(error = aMessage.Read(aOffset, leaseRecord));
    VerifyOrExit(leaseRecord.IsValid(), error = OT_ERROR_PARSE);
    aOffset += leaseRecord.GetSize();

    aHost->SetLease(leaseRecord.GetLeaseInterval());
    aHost->SetKeyLease(leaseRecord.GetKeyLeaseInterval());

    // SIG(0).
    sigOffset = aOffset;
    SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, name, sizeof(name)));
    SuccessOrExit(error = aMessage.Read(aOffset, sigRecord));
    VerifyOrExit(sigRecord.IsValid(), error = OT_ERROR_PARSE);

    sigRdataOffset = aOffset + sizeof(Dns::ResourceRecord);
    aOffset += sizeof(sigRecord);

    // TODO: verify that the signature doesn't expire.

    SuccessOrExit(error = Dns::Name::ReadName(aMessage, aOffset, aHeaderOffset, signerName, signerNameLength));

    signatureLength = sigRecord.GetLength() - (aOffset - sigRdataOffset);
    VerifyOrExit(signatureLength <= sizeof(signature), error = OT_ERROR_NO_BUFS);
    VerifyOrExit(aMessage.ReadBytes(aOffset, signature, signatureLength) == signatureLength, error = OT_ERROR_PARSE);
    aOffset += signatureLength;

    // Verify the signature. Currently supports only ECDSA.

    VerifyOrExit(sigRecord.GetAlgorithm() == Dns::KeyRecord::kAlgorithmEcdsaP256Sha256, error = OT_ERROR_FAILED);
    VerifyOrExit(signatureLength == Crypto::Ecdsa::P256::Signature::kSize, error = OT_ERROR_PARSE);
    VerifyOrExit(sigRecord.GetTypeCovered() == 0, error = OT_ERROR_FAILED);

    SuccessOrExit(error = VerifySignature(*aHost->GetKey(), aMessage, aDnsHeader, sigOffset, sigRdataOffset,
                                          sigRecord.GetLength()));

exit:
    return error;
}

otError SrpServer::VerifySignature(const Dns::Ecdsa256KeyRecord &aKey,
                                   const Message &               aMessage,
                                   Dns::Header                   aDnsHeader,
                                   uint16_t                      aSigOffset,
                                   uint16_t                      aSigRdataOffset,
                                   uint16_t                      aSigRdataLength)
{
    otError                        error;
    uint16_t                       offset = aMessage.GetOffset();
    uint16_t                       signatureOffset;
    Crypto::Sha256                 sha256;
    Crypto::Sha256::Hash           hash;
    Crypto::Ecdsa::P256::Signature signature;

    VerifyOrExit(aSigRdataLength >= Crypto::Ecdsa::P256::Signature::kSize, error = OT_ERROR_INVALID_ARGS);

    sha256.Start();

    // SIG RDATA less signature.
    sha256.Update(aMessage, aSigRdataOffset, aSigRdataLength - Crypto::Ecdsa::P256::Signature::kSize);

    // We need the DNS header before appending the SIG RR.
    aDnsHeader.SetAdditionalRecordsCount(aDnsHeader.GetAdditionalRecordsCount() - 1);
    sha256.Update(aDnsHeader);
    sha256.Update(aMessage, offset + sizeof(aDnsHeader), aSigOffset - offset - sizeof(aDnsHeader));

    sha256.Finish(hash);

    signatureOffset = aSigRdataOffset + aSigRdataLength - Crypto::Ecdsa::P256::Signature::kSize;
    SuccessOrExit(error = aMessage.Read(signatureOffset, signature));

    error = aKey.GetKey().Verify(hash, signature);

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

    if (mAdvertisingHandler != nullptr)
    {
        UpdateMetadata *update = UpdateMetadata::New(aDnsHeader, aHost, aMessageInfo);

        SuccessOrExit(error = mOutstandingUpdates.Add(*update));
        mOutstandingUpdatesTimer.StartAt(mOutstandingUpdates.GetHead()->GetExpireTime(), 0);

        mAdvertisingHandler(aHost, events, kDefaultEventsHandlerTimeout, mAdvertisingHandlerContext);
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
        otLogInfoSrp("SRP server: sent fail response: %d", aResponseCode);
    }
    else
    {
        otLogInfoSrp("SRP server: sent success response");
    }

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnSrp("SRP server: failed to send response: %s", otThreadErrorToString(error));
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

    otLogInfoSrp("SRP server: sent response with granted lease: %u and key lease: %u", aLease, aKeyLease);

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnSrp("SRP server: failed to send response: %s", otThreadErrorToString(error));
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
        otLogInfoSrp("SRP server: failed to handle DNS message: %s", otThreadErrorToString(error));
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
            otLogInfoSrp("SRP server: KEY LEASE of host %s expires", host->GetFullName());

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
                    otLogInfoSrp("SRP server: KEY LEASE of service %s expires", service->GetFullName());
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
            otLogInfoSrp("SRP server: LEASE of host %s expires", host->GetFullName());

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
                    otLogInfoSrp("SRP server: LEASE of service %s expires", service->GetFullName());

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
            otLogInfoSrp("SRP server: lease timer is scheduled in %u seconds", (earliestExpireTime - now) / 1000);
            mLeaseTimer.StartAt(earliestExpireTime, 0);
        }
    }
    else
    {
        otLogInfoSrp("SRP server: lease timer is stopped");
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
        HandleAdvertisingResult(mOutstandingUpdates.GetTail(), OT_ERROR_RESPONSE_TIMEOUT);
    }
}

SrpServer::Service *SrpServer::Service::New(const char *aFullName)
{
    void *   buf;
    Service *service = nullptr;

    buf = Instance::Get().HeapCAlloc(1, sizeof(Service));
    VerifyOrExit(buf != nullptr);

    service = new (buf) Service();
    if (service->SetFullName(aFullName) != OT_ERROR_NONE)
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
    mFullName       = nullptr;
    mPort           = 0;
    mHost           = nullptr;
    mTxtData        = nullptr;
    mTxtLength      = 0;
    mNext           = nullptr;
    mIsDeleted      = false;
    mTimeLastUpdate = TimerMilli::GetNow();
}

SrpServer::Service::~Service()
{
    Instance::Get().HeapFree(mFullName);
    Instance::Get().HeapFree(mTxtData);
}

otError SrpServer::Service::SetFullName(const char *aFullName)
{
    OT_ASSERT(aFullName != nullptr);

    otError error    = OT_ERROR_NONE;
    char *  nameCopy = static_cast<char *>(Instance::Get().HeapCAlloc(1, strlen(aFullName) + 1));

    VerifyOrExit(nameCopy != nullptr, error = OT_ERROR_NO_BUFS);
    strcpy(nameCopy, aFullName);

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

bool SrpServer::Service::Matches(const char *aFullName) const
{
    return (mFullName != nullptr) && (strcmp(mFullName, aFullName) == 0);
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
    : mFullName(nullptr)
    , mAddressesNum(0)
    , mNext(nullptr)
    , mLease(0)
    , mKeyLease(0)
    , mTimeLastUpdate(TimerMilli::GetNow())
{
    memset(&mKey, 0, sizeof(mKey));
}

SrpServer::Host::~Host()
{
    RemoveAllServices();
    Instance::Get().HeapFree(mFullName);
}

otError SrpServer::Host::SetFullName(const char *aFullName)
{
    OT_ASSERT(aFullName != nullptr);

    otError error    = OT_ERROR_NONE;
    char *  nameCopy = static_cast<char *>(Instance::Get().HeapCAlloc(1, strlen(aFullName) + 1));

    VerifyOrExit(nameCopy != nullptr, error = OT_ERROR_NO_BUFS);
    strcpy(nameCopy, aFullName);

    if (mFullName != nullptr)
    {
        Instance::Get().HeapFree(mFullName);
    }
    mFullName = nameCopy;

    mTimeLastUpdate = TimerMilli::GetNow();

exit:
    return error;
}

void SrpServer::Host::SetKey(Dns::Ecdsa256KeyRecord &aKey)
{
    OT_ASSERT(aKey.IsValid());

    mKey            = aKey;
    mTimeLastUpdate = TimerMilli::GetNow();
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

SrpServer::Service *SrpServer::Host::AddService(const char *aFullName)
{
    Service *service;

    service = FindService(aFullName);
    if (service == nullptr)
    {
        service = Service::New(aFullName);
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
    IgnoreError(mServices.Remove(*aService));
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

    if (mAddressesNum >= kMaxAddressesNum)
    {
        otLogWarnSrp("SRP server: too many addresses for host %s", GetFullName());
        ExitNow(error = OT_ERROR_NO_BUFS);
    }

    mAddresses[mAddressesNum++] = aIp6Address;
    mTimeLastUpdate             = TimerMilli::GetNow();

exit:
    return error;
}

bool SrpServer::Host::Matches(const char *aName) const
{
    return mFullName != nullptr && strcmp(mFullName, aName) == 0;
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

} // namespace Srp

} // namespace ot

#endif // OPENTHREAD_CONFIG_SRP_SERVER_ENABLE
