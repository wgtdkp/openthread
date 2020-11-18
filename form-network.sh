#!/bin/bash

readonly NETWORK_NAME=openthread-test
readonly CHANNEL=19
readonly PANID=0xface
readonly XPANID=dead00beef00cafe
readonly MASTERKEY=00112233445566778899aabbccddeeff
readonly PASSWORD=123456
readonly OT_CTL=./cmake-build/src/posix/ot-ctl

sudo "${OT_CTL}" factoryreset
sleep 1

sudo "${OT_CTL}" networkname "${NETWORK_NAME}"
sudo "${OT_CTL}" channel "${CHANNEL}"
sudo "${OT_CTL}" panid "${PANID}"
sudo "${OT_CTL}" extpanid "${XPANID}"
sudo "${OT_CTL}" pskc -p "${PASSWORD}"
sudo "${OT_CTL}" masterkey "${MASTERKEY}"
sudo "${OT_CTL}" ifconfig up
sudo "${OT_CTL}" thread start

sleep 5
sudo "${OT_CTL}" state
