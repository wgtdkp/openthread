# OpenThread CLI - ToBLE Platform Example

The OpenThread ToBLE Platform APIs will be invoked via the OpenThread CLI.

## Quick Start
### Build
```
make -f ./examples/Makefile-nrf52840 TOBLE=1 DISABLE_BUILTIN_MBEDTLS=0
```

### Donwload nRF5 SDK
```
wget https://www.nordicsemi.com/-/media/Software-and-other-downloads/SDKs/nRF5/Binaries/nRF5SDK17009d13099.zip
unzip nRF5SDK17009d13099.zip
```

### Flash softdevice and application

Flash the softdevice:
```
nrfjprog -f nrf52 -s ${device_sn} --program ${path-to-nRF5SDK}/components/softdevice/s140/hex/s140_nrf52_7.0.1_softdevice.hex --sectorerase
nrfjprog -f nrf52 -s ${device_sn} --reset
```

Flash the application:
```
arm-none-eabi-objcopy -O ihex <path-to-openthread>/output/nrf52840/bin/ot-cli-ftd <path-to-openthread>/output/nrf52840/bin/ot-cli-ftd.hex

nrfjprog -f nrf52 -s ${device_sn} --program <path-to-openthread>/output/nrf52840/bin/ot-cli-ftd.hex --sectorerase
nrfjprog -f nrf52 -s ${device_sn} --reset
```

## Examples

### On Node1:
```bash
>tobleplat diag start
Done
>tobleplat adv start 100
Done
```

### On Node 2:

```bash
>tobleplat diag start
Done
> tobleplat scan start 120 30 0
|     advType     | addrType |   address    | rssi | AD or Scan Rsp Data |
+-----------------+----------+--------------+------+---------------------|
Done
> | ADV_NONCONN_IND |    3     | 15e513dfbb21 | -45  | 1eff0600010f2002c6abcc8cd752a5b73a9eb88e5bd34fe0c7c49722990245
| ADV_IND         |    1     | a8819b1a7dd0 | -17  | 0201061216fbff300000010002fc1122334455667788
| ADV_SCAN_IND    |    2     | 6ec3dec87c79 | -47  | 1eff4c000719010e2000f88f000004b5c11df83a337de4229817901b71c7d7
| ADV_NONCONN_IND |    3     | fac830245e1e | -44  | 1eff0600010f2002af9d1bd6ae406b79204cf432067bdec901822959061fdc
| ADV_IND         |    2     | d6ce1d889249 | -61  | 02011a020a0c0aff4c00100503186c8621
| ADV_SCAN_IND    |    2     | f98f2bb23c52 | -70  | 03039ffe17169ffe0000000000000000000000000000000000000000
| ADV_IND         |    2     | 1019c14ee27c | -67  | 0201020303f3fe
| SCAN_RSP        |    2     | 1019c14ee27c | -67  | 1b16f3fe5141417741414141414149444141437535426d6f41414141
| ADV_IND         |    2     | 5405d418d640 | -52  | 02011a020a0c0bff4c0010060d1ab91f6af7

>tobleplat scan stop
Done
>
>tobleplat role central
Done
>
>tobleplat connect start 1 a8819b1a7dd0
Done
>HandleConnected: index=0, platConn=0x2001e634
HandleConnectionIsReady: index=0, linkType=Gatt
>
> tobleplat show

| Index |     State     |   Address    |  PlatConn  |
+-------+---------------+--------------|------------|
| 0     | Ready         | a8819b1a7dd0 | 0x2001e634 |
Done
>
> tobleplat subscribe 0 1
Done
>
> tobleplat mtu 0
23
Done
>
> tobleplat send 0 20
Done
> HndleC1WriteDone: index=0
>
```
At the same time, the Node 1 outputs the following information:
```
HandleConnected: index=0, platConn=0x2001e634
HandleC2Subscribed: index=0, isSubscribed=True
HandleC1Write: index=0, length=20
```

## Command List

- [help](#help)
- [diag start](#diag-start)
- [diag stop](#diag-stop)
- [adv start](#adv-start)
- [adv stop](#adv-stop)
- [connect start](#connect-start)
- [connect stop](#connect-stop)
- [link](#link)
- [mtu](#mtu)
- [role](#role)
- [scan start](#scan-start)
- [scan stop](#scan-stop)
- [send](#send)
- [show](#show)
- [subscribe](#subscribe)

## Command Details

### help

Usage: `tobleplat help`

Print dataset help menu.

```bash
> tobleplat help
help
diag
adv
scan
connect
disconnect
mtu
subscribe
send
role
link
show
Done
```

### diag start

Usage: `tobleplat diag start`

Enable the ToBLE platform diag API.

```bash
> tobleplat diag start
Done
```

### diag stop

Usage: `tobleplat diag stop`

Disable the ToBLE platform diag API.

```bash
> tobleplat diag stop
Done
```

### adv start

Usage: `tobleplat adv start <interval>`

Start to send BLE advertising packets.

- interval: BLE advertising interval, in ms.

```bash
> tobleplat adv start 100
Done
```

### adv stop

Usage: `tobleplat adv stop`

Stop sending BLE advertising packets.

```bash
> tobleplat adv stop
Done
```

### scan start

Usage: `tobleplat scan start <interval> <window> <active>`

Start to scan BLE advertising packets.

- interval: BLE scan interval, in ms.
- window: BLE scan window, in ms.
- active: 0: passive scan, 1: active scan.

```bash
> tobleplat scan start 120 30 1
|     advType     | addrType |   address    | rssi | AD or Scan Rsp Data |
+-----------------+----------+--------------+------+---------------------|
Done
> | ADV_NONCONN_IND |    3     | 15e513dfbb21 | -45  | 1eff0600010f2002c6abcc8cd752a5b73a9eb88e5bd34fe0c7c49722990245
| ADV_IND         |    1     | a8819b1a7dd0 | -17  | 0201061216fbff300000010002fc1122334455667788
| ADV_SCAN_IND    |    2     | 6ec3dec87c79 | -47  | 1eff4c000719010e2000f88f000004b5c11df83a337de4229817901b71c7d7
| ADV_NONCONN_IND |    3     | fac830245e1e | -44  | 1eff0600010f2002af9d1bd6ae406b79204cf432067bdec901822959061fdc
| ADV_IND         |    2     | d6ce1d889249 | -61  | 02011a020a0c0aff4c00100503186c8621
| ADV_SCAN_IND    |    2     | f98f2bb23c52 | -70  | 03039ffe17169ffe0000000000000000000000000000000000000000
| ADV_IND         |    2     | 1019c14ee27c | -67  | 0201020303f3fe
| SCAN_RSP        |    2     | 1019c14ee27c | -67  | 1b16f3fe5141417741414141414149444141437535426d6f41414141
| ADV_IND         |    2     | 5405d418d640 | -52  | 02011a020a0c0bff4c0010060d1ab91f6af7
```

### scan stop

Usage: `tobleplat scan stop`

Stop BLE scanning.

```bash
> tobleplat scan stop
Done
```

### connect start

Usage: `tobleplat connect start <addrType> <address>`

Create a ToBLE connection with the peripheral.

- addrType: BLE address type.
- address: BLE address.

```bash
> tobleplat connect start 1 22ef951da7dc
Done
>HandleConnected: index=1, platConn=0x2001e640
HandleConnectionIsReady: index=1, linkType=Gatt
```

### show

Usage: `tobleplat show`

Show ToBLE connections.

```bash
> tobleplat show

| Index |     State     |   Address    |  PlatConn  |
+-------+---------------+--------------|------------|
| 0     | Ready         | a8819b1a7dd0 | 0x2001e634 |
| 1     | Ready         | 22ef951da7dc | 0x2001e640 |
```

### connect stop

Usage: `tobleplat connect stop <connectionIndex>`

Disconnect a ToBLE connection.

- connectionIndex: The index of the ToBLE connection.

```bash
> tobleplat disconnect 0
Done
>HandleDisconnected: platConn=0x2001e634
HandleDisconnected: index=0, platConn=0x2001e634
```

### mtu

Usage: `tobleplat mtu <connectionIndex>`

Get the MTU of the ToBLE connection.

- connectionIndex: The index of the ToBLE connection.

```bash
> tobleplat mtu 0
23
Done
```

### subscribe

Usage: `tobleplat subscribe <connectionIndex> <isSubscribe>`

Request a subscription change to C2 (BTP) on a given ToBLE connection.

- connectionIndex: The index of the ToBLE connection.
- subscribe: 0: un-subscribe, 1: subscribe.

```bash
> tobleplat subscribe 0 1
Done
```

### send

Usage: `tobleplat send <connectionIndex> <length>`

Send a packet on the specified ToBLE connection.

- connectionIndex: The index of the ToBLE connection.
- length: The length of the packet.

```bash
> tobleplat send 0 20
Done
> HandleC1WriteDone: index=0
```

### link

Usage: `tobleplat link <linkType>`

Set or show the ToBLE link type.

- linkType: "gatt": GATT BTP link, "l2cap": L2CAP link.

```bash
> tobleplat link gatt
Done
> tobleplat link
gatt
Done
```

### role

Usage: `tobleplat role <roleType>`

Set or show the BLE role.

- roleType: "central": BLE central device, "peripheral": BLE peripheral device.

```bash
> tobleplat role central
Done
> tobleplat role
central
Done
```
