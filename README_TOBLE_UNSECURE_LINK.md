# OpenThread TOBLE Unsecure Link Demo

This README shows how to do a unsecure link demo based on ToBLE link.

## Quick Start
### Get Source Code
```
git clone --branch toble-1 sso://user/zhanglongxia/my-openthread
cd my-openthread/
```

### Build nRF52840
```
make -f ./examples/Makefile-nrf52840 TOBLE=1 DISABLE_BUILTIN_MBEDTLS=0
```

#### Donwload nRF5 SDK
```
wget https://www.nordicsemi.com/-/media/Software-and-other-downloads/SDKs/nRF5/Binaries/nRF5SDK17009d13099.zip
unzip nRF5SDK17009d13099.zip
```

#### Flash softdevice and application

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

### Build Posix App

#### Install libraries
```
sudo apt-get update
sudo apt-get -y install libglib2.0-dev libdbus-glib-1-dev libudev-dev libical-dev libreadline-dev
```

#### Build
```
./bootstrap
make -f ./src/posix/Makefile-posix TOBLE=1 BLE_HOST=bluez
```

#### Run Posix App
Get the HCI index
```
$ hciconfig
hci0:   Type: Primary  Bus: USB
    BD Address: 00:19:86:00:0F:7B  ACL MTU: 1021:8  SCO MTU: 64:1
    UP RUNNING
    RX bytes:1542 acl:0 sco:0 events:79 errors:0
    TX bytes:2081 acl:0 sco:0 commands:79 errors:0
```

Run Posix App
```
sudo ./output/posix/x86_64-unknown-linux-gnu/bin/ot-cli hci0
```

### Unsecure Ping example
#### On Node 1
```
>panid 1
Done
>ifconfig up
Done
> ipaddr
fe80:0:0:0:ecdd:1827:58c9:8254
Done
```

#### On Node 2
```
>toble linkmode central
Done
>panid 1
Done
> ifconfig up
Done
> ping fe80:0:0:0:ecdd:1827:58c9:8254
Done
> 16 bytes from fe80:0:0:0:ecdd:1827:58c9:8254: icmp_seq=1 hlim=64 time=3127ms
```


### Unsecure UDP example
#### On Node 1

Start IPv6 interface and open UDP server.
```
>panid 1
Done
>ifconfig up
Done
> ipaddr
fe80:0:0:0:e47b:1166:48e9:738e
Done
> unsecureport add 1234
Done
> udp open
Done
> udp bind :: 1234
Done
```

#### On Node 2

Start IPv6 interface and send a UDP packet to the peer.
```
>toble linkmode central
Done
>panid 1
Done
> ifconfig up
Done
> udp linksecurity disable
Done
> udp open
Done
> udp send fe80:0:0:0:e47b:1166:48e9:738e 1234 hello
Done
```

#### On Node 1
The UDP server receives a packet.
```
> 5 bytes from fe80:0:0:0:b881:ed10:9a25:b889 49152 hello
```
