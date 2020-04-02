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
 *   This file implements lwIP Network Interface with OpenThread.
 */

#include <arpa/inet.h>
#include <assert.h>
#include <lwip/netif.h>
#include <lwip/priv/tcpip_priv.h>
#include <lwip/tcpip.h>
#include <lwip/udp.h>

#include "esp32_platform.h"

#include <openthread/icmp6.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/message.h>
#include <openthread/thread.h>
#include <openthread/platform/radio.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"
#include "event_queue_api.h"
#include "string.h"

#if OPENTHREAD_CONFIG_PLATFORM_NETIF_ENABLE
//static void (*otPlatformNetifEventHandler)(otInstance*, const otPlatNetifEvent*);
//static otPlatNetifEventHandler sEventHandler = NULL;
static const size_t kMaxIp6Size = 1500;

static struct netif sNetif;

/**
 * This structure stores information needed to send a IPv6 packet.
 *
 */
struct OutputEvent {
  uint16_t mLength;
  uint8_t mData[kMaxIp6Size];
};

static bool IsLinkLocal(const struct otIp6Address &aAddress) {
  return aAddress.mFields.m16[0] == htons(0xfe80);
}

static void netifEvent(struct netif *aNetif) {
  // TODO must use queue
}

static err_t NetIfOutputIp6(struct netif *aNetif, struct pbuf *aBuffer,
                            const ip6_addr_t *aPeerAddr) {
  err_t err = ERR_OK;
  OutputEvent *event = NULL;

  otLogInfoPlat("netif output");
  assert(aNetif == &sNetif);
  event = (OutputEvent *)malloc(sizeof(*event) + aBuffer->tot_len);

  event->mLength = aBuffer->tot_len;
  VerifyOrExit(event != NULL, err = ERR_BUF);
  VerifyOrExit(aBuffer->tot_len == pbuf_copy_partial(aBuffer, event->mData,
                                                     aBuffer->tot_len, 0),
               err = ERR_ARG);
  VerifyOrExit(OT_ERROR_NONE ==
                   otEventQueuePush(static_cast<otInstance *>(aNetif->state),
                                    EVENT_NETIF_OUTPUT, event, 0),
               err = ERR_IF);

exit:
  if (err != ERR_OK) {
    if (event != NULL) {
      free(event);
    }
  }
  return err;
}

static err_t NetIfInit(struct netif *aNetif) {
  aNetif->name[0] = 'o';
  aNetif->name[1] = 't';
  aNetif->hwaddr_len = sizeof(otExtAddress);
  memset(aNetif->hwaddr, 0, sizeof(aNetif->hwaddr));
  aNetif->mtu = OPENTHREAD_CONFIG_IP6_MAX_DATAGRAM_LENGTH;
  aNetif->flags = NETIF_FLAG_BROADCAST;
  aNetif->output = NULL;
#if LWIP_IPV6
  aNetif->output_ip6 = NetIfOutputIp6;
#endif
  aNetif->num = 0;

  return ERR_OK;
}

struct netif *otxGetNetif(void) {
  return &sNetif;
}

// wgtdkp: when is this used?
void otPlatNetifInit(otInstance *aInstance) {
  otError error = OT_ERROR_FAILED;

  memset(&sNetif, 0, sizeof(sNetif));
  LOCK_TCPIP_CORE();
  netif_add(&sNetif, NULL, NULL, NULL, aInstance, NetIfInit, tcpip_input);
  netif_set_link_up(&sNetif);
  netif_set_status_callback(&sNetif, netifEvent);
  UNLOCK_TCPIP_CORE();

//  sEventHandler = aHandler;
  error = OT_ERROR_NONE;

  if (error != OT_ERROR_NONE) {
    perror("init netif");
    otLogCritPlat(aInstance, "failed to initialize netif %d", error);
  }
}

otError otPlatNetifUp(otInstance *aInstance) {
  LOCK_TCPIP_CORE();
  netif_set_up(&sNetif);
  UNLOCK_TCPIP_CORE();

  return OT_ERROR_NONE;
}

otError otPlatNetifDown(otInstance *aInstance) {
  LOCK_TCPIP_CORE();
  netif_set_down(&sNetif);
  UNLOCK_TCPIP_CORE();

  return OT_ERROR_NONE;
}

otError otPlatNetifAddAddress(otInstance *aInstance,
                              const otNetifAddress *aNetifAddress) {
  otError error = OT_ERROR_NONE;
  err_t err = ERR_OK;

  OT_UNUSED_VARIABLE(aInstance);

  LOCK_TCPIP_CORE();
  if (IsLinkLocal(aNetifAddress->mAddress)) {
    netif_ip6_addr_set(
        &sNetif, 0,
        reinterpret_cast<const ip6_addr_t *>(&aNetifAddress->mAddress));
    netif_ip6_addr_set_state(&sNetif, 0, IP6_ADDR_PREFERRED);
  } else {
    int8_t index = -1;
    err = netif_add_ip6_address(
        &sNetif, reinterpret_cast<const ip6_addr_t *>(&aNetifAddress->mAddress),
        &index);
    VerifyOrExit(err == ERR_OK && index != -1, error = OT_ERROR_FAILED);
    netif_ip6_addr_set_state(&sNetif, index, IP6_ADDR_PREFERRED);
  }

exit:
  UNLOCK_TCPIP_CORE();

  if (error != OT_ERROR_NONE) {
    otLogCritPlat(aInstance, "failed to add address: %d", error);
  }

  return error;
}

otError otPlatNetifRemoveAddress(otInstance *aInstance,
                                 const otNetifAddress *aNetifAddress) {
  otError error = OT_ERROR_NONE;
  int8_t index;

  OT_UNUSED_VARIABLE(aInstance);

  LOCK_TCPIP_CORE();
  index = netif_get_ip6_addr_match(
      &sNetif, reinterpret_cast<const ip6_addr_t *>(&aNetifAddress->mAddress));
  VerifyOrExit(index != -1, error = OT_ERROR_NOT_FOUND);
  netif_ip6_addr_set_state(&sNetif, index, IP6_ADDR_INVALID);

exit:
  UNLOCK_TCPIP_CORE();
  return error;
}

void otPlatNetifReceive(otInstance *aInstance, otMessage *aMessage) {
  otError error = OT_ERROR_NONE;
  err_t err = ERR_OK;
  const size_t kBlockSize = 128;
  uint16_t length = otMessageGetLength(aMessage);
  struct pbuf *buffer = NULL;

  OT_UNUSED_VARIABLE(aInstance);

  printf("ot receive to lwip\n");
  buffer = pbuf_alloc(PBUF_RAW, length, PBUF_POOL);

  VerifyOrExit(buffer != NULL, error = OT_ERROR_NO_BUFS);

  for (uint16_t i = 0; i < length; i += kBlockSize) {
    uint8_t block[kBlockSize];
    int count = otMessageRead(aMessage, i, block, sizeof(block));

    assert(count > 0);
    err = pbuf_take_at(buffer, block, count, i);
    VerifyOrExit(err == ERR_OK, error = OT_ERROR_FAILED);
  }

  VerifyOrExit(ERR_OK == sNetif.input(buffer, &sNetif),
               error = OT_ERROR_FAILED);

exit:
  if (error != OT_ERROR_NONE) {
    if (buffer != NULL) {
      pbuf_free(buffer);
    }

    if (error == OT_ERROR_FAILED) {
      otLogWarnPlat(aInstance, "%s failed for lwip error %d", __func__, err);
    }

    otLogWarnPlat(aInstance, "%s failed: %s", __func__,
                  otThreadErrorToString(error));
  }

  otMessageFree(aMessage);
}

static void processTransmit(otInstance *aInstance, const OutputEvent &aEvent) {
  otError error = OT_ERROR_NONE;
  otMessage *message = NULL;

  {
    message = otIp6NewMessage(aInstance, NULL);
    VerifyOrExit(message != NULL, error = OT_ERROR_NO_BUFS);

    SuccessOrExit(error =
                      otMessageAppend(message, aEvent.mData, aEvent.mLength));

    error = otIp6Send(aInstance, message);
    message = NULL;
  }

exit:
  if (error != OT_ERROR_NONE) {
    if (message != NULL) {
      otMessageFree(message);
    }

    otLogWarnPlat(aInstance, "Failed to transmit IPv6 packet: %s",
                  otThreadErrorToString(error));
  }
}

void esp32NetifProcess(otInstance *aInstance, int aType, const void *aData) {
//  VerifyOrExit(sEventHandler != NULL);

  switch (aType) {
    case EVENT_NETIF_OUTPUT:
      otLogInfoPlat(aInstance, "esp32NetifProcess: aType=EVENT_NETIF_OUTPUT");
      processTransmit(aInstance, *static_cast<const OutputEvent *>(aData));
      break;
//    case EVENT_NETIF_CHANGED:
//      sEventHandler(aInstance, static_cast<const otPlatNetifEvent *>(aData));
//      break;
    default:
      break;
  }

}


#endif  // OPENTHREAD_CONFIG_PLATFORM_NETIF_ENABLE
