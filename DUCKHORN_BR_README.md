# Duchorn Border Router

## Build

There are two scripts for building the Border Router (`ot-daemon` + `ot-rcp`) and End Device (`ot-cli-ftd`)

- [make-daemon.sh](./make-daemon.sh) create `ot-daemon` and `ot-ctl` inside dir `.cmake-build-daemon/src/posix`.
- [make-nrf5-cli-rcp.sh](./make-daemon.sh) create `ot-rcp` and `ot-cli-ftd` inside dir `output/nrf52840/bin`.

## Flash devices

We use nRF52840 dongle within this guide. Flash your RCP device with [nrfutil](https://infocenter.nordicsemi.com/topic/ug_nrfutil/UG/nrfutil/nrfutil_intro.html).

```shell
nrfutil dfu serial -pkg output/nrf52840/bin/ot-rcp.zip  -p /dev/ttyACM0
nrfutil dfu serial -pkg output/nrf52840/bin/ot-cli-ftd.zip  -p /dev/ttyACM1
```

## Testing

1. Start the Border Router:

    ```shell
    sudo ./.cmake-build-daemon/src/posix/ot-daemon -B wlan0 -v 'spinel+hdlc+uart:///dev/ttyACM0?uart-baudrate=115200'
    ```

2. Form a new Thread Network with script [form-network.sh](./form-network.sh):

    ```shell
    sudo ./form-network.sh
    ```

3. Attch the End Device to the new Thread Network:

    ```shell
    screen /dev/ttyACM1
    > factoryreset
    Done
    > panid 0xface
    Done
    > channel 19
    Done
    > ifconfig up
    Done
    > thread start
    Done
    > ipaddr
    fdde:ad00:beef:0:0:ff:fe00:c400
    fd33:89bc:362:0:3cef:e398:410d:a9d5
    fdde:ad00:beef:0:e69e:4560:4d8e:c160
    fe80:0:0:0:b0f9:4a19:aa56:e205
    Done
    ```

4. On the Border Router, we can find an address `inet6 fd33:89bc:362:3::1/64 scope global nodad` on `wlan0`. We can ping this address from the End Device:

    ```shell
    > ping fd33:89bc:362:3::1
    Done
    > 16 bytes from fd33:89bc:362:3:0:0:0:1: icmp_seq=1 hlim=64 time=12ms
    ```
