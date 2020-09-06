# OpenThread TOBLE MeshCop Commissioning Demo

This README shows how to do a ToBLE MeshCop Commissioning demo based on ToBLE link.

## Quick Start
### Build
#### nRF52840
```
./bootstrap
make -f ./examples/Makefile-nrf52840 TOBLE=1 DISABLE_BUILTIN_MBEDTLS=0 JOINER=1 COMMISSIONER=1
```

##### Donwload nRF5 SDK
```
wget https://www.nordicsemi.com/-/media/Software-and-other-downloads/SDKs/nRF5/Binaries/nRF5SDK17009d13099.zip
unzip nRF5SDK17009d13099.zip
```

##### Flash softdevice and application

Flash the softdevice:
```
nrfjprog -f nrf52 -s ${device_sn} --program ${path-to-nRF5SDK}/components/softdevice/s140/hex/s140_nrf52_7.0.1_softdevice.hex --sectorerase
nrfjprog -f nrf52 -s ${device_sn} --reset
```

Flash the application of the Node 1 (Node 1 is FTD):
```
arm-none-eabi-objcopy -O ihex <path-to-openthread>/output/nrf52840/bin/ot-cli-ftd <path-to-openthread>/output/nrf52840/bin/ot-cli-ftd.hex

nrfjprog -f nrf52 -s ${device_sn} --program <path-to-openthread>/output/nrf52840/bin/ot-cli-ftd.hex --sectorerase
nrfjprog -f nrf52 -s ${device_sn} --reset
```

Flash the application of the Node 2 (Node 2 is MTD):
```
arm-none-eabi-objcopy -O ihex <path-to-openthread>/output/nrf52840/bin/ot-cli-ftd <path-to-openthread>/output/nrf52840/bin/ot-cli-mtd.hex

nrfjprog -f nrf52 -s ${device_sn} --program <path-to-openthread>/output/nrf52840/bin/ot-cli-mtd.hex --sectorerase
nrfjprog -f nrf52 -s ${device_sn} --reset
```

#### Posix
Install libraries
```
sudo apt-get update
sudo apt-get -y install libglib2.0-dev libdbus-glib-1-dev libudev-dev libical-dev libreadline-dev
```

Compile
```
./bootstrap
make -f ./src/posix/Makefile-posix TOBLE=1 BLE_HOST=bluez
```

Get the HCI index
```
$ hciconfig
hci0:   Type: Primary  Bus: USB
    BD Address: 00:19:86:00:0F:7B  ACL MTU: 1021:8  SCO MTU: 64:1
    UP RUNNING
    RX bytes:1542 acl:0 sco:0 events:79 errors:0
    TX bytes:2081 acl:0 sco:0 commands:79 errors:0
```

Run Toble
```
sudo ./output/posix/x86_64-unknown-linux-gnu/bin/ot-cli hci0
```

## Example

### On Node 1

Start FTD.
```
> toble linkmode central
Done
> 
> panid 1
Done
> 
> ifconfig up
Done
> 
> thread start
Done
> 
> state
leader
>
> commissioner start
Commissioner: petitioning
Done
> Commissioner: active
>
> commissioner joiner add * J01NME
Done
>
```

### On Node 2

Start joiner.
```
ifconfig up
Done
> joiner start J01NME
Done
Join success
> 
```

Wait about 100 seconds.
```
> thread start
Done
> state
child
Done
```
