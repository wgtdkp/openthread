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
 *   This file implements a SRP client.
 */

#include "srp_client.hpp"

#if OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE

#include <openthread/ip6.h>
#include <openthread/netdata.h>

#include "common/instance.hpp"
#include "common/locator-getters.hpp"

#include "srp-thread.h"

#define DNS_RRTYPE_AAAA (28)

void* create_timer(otInstance* aInstance, srp_wakeup_callback_t aCallback)
{
  ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);
  instance.Get<ot::SrpClient>().SetTimerHandler(aCallback);
  return instance.Get<ot::SrpClient>().GetTimer();
}

void start_timer(otInstance* aInstance, void *aTimer, uint32_t aMilliseconds)
{
  ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);
  instance.Get<ot::SrpClient>().GetTimer()->Start(aMilliseconds);
}

void stop_timer(otInstance* aInstance, void *aTimer)
{
  ot::Instance &instance = *static_cast<ot::Instance *>(aInstance);
  instance.Get<ot::SrpClient>().GetTimer()->Stop();
}

namespace ot {

static bool IsSrpService(otServiceConfig *serviceConfig)
{
  return serviceConfig->mEnterpriseNumber == THREAD_ENTERPRISE_NUMBER &&
         serviceConfig->mServiceDataLength == 1 &&
         serviceConfig->mServiceData[0] == THREAD_SRP_SERVER_OPTION &&
         serviceConfig->mServerConfig.mServerDataLength == OT_IP6_ADDRESS_SIZE + sizeof(uint16_t);
}

otError SrpClient::Start(void)
{
  int ret;
  otError error = OT_ERROR_NONE;
  otNetworkDataIterator iterator = OT_NETWORK_DATA_ITERATOR_INIT;
  const otNetifAddress *netifAddr;
  otServiceConfig serviceConfig;

  VerifyOrExit(srp_thread_init(&GetInstance()) == 0, error = OT_ERROR_NONE);

  netifAddr = otIp6GetUnicastAddresses(&GetInstance());
  while (netifAddr != nullptr)
  {
    ret = srp_add_interface_address(DNS_RRTYPE_AAAA, netifAddr->mAddress.mFields.m8, sizeof(netifAddr->mAddress));
    OT_ASSERT(ret == 0);
    netifAddr = netifAddr->mNext;
  }

  while (otNetDataGetNextService(&GetInstance(), &iterator, &serviceConfig) == OT_ERROR_NONE)
  {
    if (IsSrpService(&serviceConfig)) {
      ret = srp_add_server_address(&serviceConfig.mServerConfig.mServerData[16],
                                   DNS_RRTYPE_AAAA, serviceConfig.mServerConfig.mServerData,
                                   serviceConfig.mServerConfig.mServerDataLength - sizeof(uint16_t));
      OT_ASSERT(ret == 0);
    }
  }

  srp_network_state_stable(nullptr);

exit:
  return error;
}

otError SrpClient::Stop(void)
{
    int error = srp_thread_shutdown(&GetInstance());
    return error == 0 ? OT_ERROR_NONE : OT_ERROR_FAILED;
}

// TODO(wgtdkp): support TXT records.
otError SrpClient::Register(const char *aName, const char *aType, const char *aDomain,
                    const char *aHost, uint16_t aPort)
{
    otError error = OT_ERROR_NONE;
    DNSServiceErrorType serviceError;
    DNSServiceRef serviceRef;

    serviceError = DNSServiceRegister(&serviceRef, 0, 0, aName, aType, aDomain, aHost, aPort, 0, nullptr, register_callback, nullptr);
    VerifyOrExit(serviceError == kDNSServiceErr_NoError, error = OT_ERROR_FAILED);
    srp_network_state_stable(nullptr);

exit:
    return error;
}

// Deregister all services.
otError SrpClient::Deregister(void)
{
    otError error = OT_ERROR_NONE;

    // TODO(wgtdkp):
    return OT_ERROR_NOT_IMPLEMENTED;
}

void SrpClient::HandleTimer(Timer &aTimer)
{
  SrpClient &srpClient = aTimer.GetOwner<SrpClient>();
  if (srpClient.mWakeupCallback)
  {
    srpClient.mWakeupCallback(&aTimer);
  }
}

} // namespace ot

#endif // OPENTHREAD_CONFIG_SRP_CLIENT_ENABLE
