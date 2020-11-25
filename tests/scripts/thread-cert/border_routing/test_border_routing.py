#!/usr/bin/env python3
#
#  Copyright (c) 2020, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
import logging
import unittest

import config
import thread_cert

# Test description: Here is the test case `5.11.1 DUA-TC-04: DUA re-registration`
#
# Topology:
#    -----------(eth)----------------
#           |           |    |
#          BR1         BR2  HOST
#           |           |
#          ED1         ED2
#

BR1 = 1
ROUTER1 = 2
BR2 = 3
ROUTER2 = 4
HOST = 5

CHANNEL1 = 18
CHANNEL2 = 19

class BorderRouting(thread_cert.TestCase):
    USE_MESSAGE_FACTORY = False

    TOPOLOGY = {
        BR1: {
            'name': 'BR_1',
            'allowlist': [ROUTER1],
            'is_otbr': True,
            'version': '1.2',
            'channel': CHANNEL1,
            'router_selection_jitter': 1,
        },
        ROUTER1: {
            'name': 'Router_1',
            'allowlist': [BR1],
            'version': '1.2',
            'channel': CHANNEL1,
            'router_selection_jitter': 1,
        },
        BR2: {
            'name': 'BR_2',
            'allowlist': [ROUTER2],
            'is_otbr': True,
            'version': '1.2',
            'channel': CHANNEL2,
            'router_selection_jitter': 1,
        },
        ROUTER2: {
            'name': 'Router_2',
            'allowlist': [BR2],
            'version': '1.2',
            'channel': CHANNEL2,
            'router_selection_jitter': 1,
        },
        HOST: {
            'name': 'Host',
            'is_host': True
        },
    }

    def test(self):
        self.nodes[HOST].start()
        # P1: Router_1 is configured with leader weight of 72 in case the test is executed on a CCM network

        self.nodes[BR1].start()
        self.simulator.go(5)
        self.assertEqual('leader', self.nodes[BR1].get_state())

        self.nodes[ROUTER1].start()
        self.simulator.go(5)
        self.assertEqual('router', self.nodes[ROUTER1].get_state())

        self.nodes[BR2].start()
        self.simulator.go(5)
        self.assertEqual('leader', self.nodes[BR2].get_state())

        self.nodes[ROUTER2].start()
        self.simulator.go(5)
        self.assertEqual('router', self.nodes[ROUTER2].get_state())

        self.simulator.go(10)  # must wait for DUA_DAD_REPEATS to complete
        logging.info("Host addresses: %r", self.nodes[HOST].get_addrs())

        self.collect_ipaddrs()

        logging.info("BR1     addrs: %r", self.nodes[BR1].get_addrs())
        logging.info("ROUTER1 addrs: %r", self.nodes[ROUTER1].get_addrs())
        logging.info("BR2     addrs: %r", self.nodes[BR2].get_addrs())
        logging.info("ROUTER2 addrs: %r", self.nodes[ROUTER2].get_addrs())

        self.assertTrue(len(self.nodes[BR1].get_prefixes()) == 1)
        self.assertTrue(len(self.nodes[ROUTER1].get_prefixes()) == 1)
        self.assertTrue(len(self.nodes[BR2].get_prefixes()) == 1)
        self.assertTrue(len(self.nodes[ROUTER2].get_prefixes()) == 1)

        self.assertTrue(len(self.nodes[ROUTER1].get_ip6_address(config.ADDRESS_TYPE.OMR)) == 1)
        self.assertTrue(len(self.nodes[ROUTER2].get_ip6_address(config.ADDRESS_TYPE.OMR)) == 1)

        self.assertTrue(self.nodes[ROUTER1].ping(self.nodes[ROUTER2].get_ip6_address(config.ADDRESS_TYPE.OMR)[0]))
        self.assertTrue(self.nodes[ROUTER2].ping(self.nodes[ROUTER1].get_ip6_address(config.ADDRESS_TYPE.OMR)[0]))

if __name__ == '__main__':
    unittest.main()