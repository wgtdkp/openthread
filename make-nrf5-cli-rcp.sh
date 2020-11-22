#!/bin/bash

DEVICE=nrf52840
DIST=nrf52840

./bootstrap
make -f examples/Makefile-nrf52840 \
    USB=1 \
    BOOTLOADER=1 \
    COAP=1 \
    COAPS=1 \
    COMMISSIONER=1 \
    BORDER_ROUTER=1 \
    BORDER_AGENT=1 \
    BACKBONE_ROUTER=0 \
    THREAD_VERSION=1.1 \
    SERVICE=1 \
    JOINER=1

arm-none-eabi-objcopy -O ihex output/${DIST}/bin/ot-rcp output/${DIST}/bin/ot-rcp.hex
arm-none-eabi-objcopy -O ihex output/${DIST}/bin/ot-cli-ftd output/${DIST}/bin/ot-cli-ftd.hex

if [ ! -f output/${DIST}/bin/private.pem ]; then
    nrfutil keys generate output/${DIST}/bin/private.pem
fi

nrfutil pkg generate --debug-mode --hw-version 52 --sd-req 0 \
    --application output/nrf52840/bin/ot-rcp.hex \
    --key-file output/${DIST}/bin/private.pem \
    output/nrf52840/bin/ot-rcp.zip

nrfutil pkg generate --debug-mode --hw-version 52 --sd-req 0 \
    --application output/nrf52840/bin/ot-cli-ftd.hex \
    --key-file output/${DIST}/bin/private.pem \
    output/nrf52840/bin/ot-cli-ftd.zip

# nrfutil dfu serial -pkg output/nrf52840/bin/ot-rcp.zip  -p /dev/ttyACM0
