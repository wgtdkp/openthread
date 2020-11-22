# Duchorn Border Router

## Devices
Please prepare yourself below hardwares:
- Two Raspberry Pi 3/4
- Four nRF52840 dongle

## Build

There are two scripts for building the Border Router (`ot-daemon` + `ot-rcp`) and End Device (`ot-cli-ftd`)

- [make-daemon.sh](./make-daemon.sh) create `ot-daemon` and `ot-ctl` inside dir `cmake-build-daemon/src/posix`.
- [make-nrf5-cli-rcp.sh](./make-nrf5-cli-rcp.sh) create `ot-rcp` and `ot-cli-ftd` inside dir `output/nrf52840/bin`.

## Flash devices

Flash your RCP & CLI devices with [nrfutil](https://infocenter.nordicsemi.com/topic/ug_nrfutil/UG/nrfutil/nrfutil_intro.html).

```shell
nrfutil dfu serial -pkg output/nrf52840/bin/ot-rcp.zip  -p /dev/ttyACM0
nrfutil dfu serial -pkg output/nrf52840/bin/ot-cli-ftd.zip  -p /dev/ttyACM1
```

## Testing

We can now test bi-drectional connectivity by pinging End Devices from different Thread network to each other (connected via WiFi).

1. Start the Border Router 1:

    ```shell
    sudo ./cmake-build-daemon/src/posix/ot-daemon -B wlan0 -v 'spinel+hdlc+uart:///dev/ttyACM0?uart-baudrate=115200'
    ```

2. Form a new Thread Network with script [form-network.sh](./form-network.sh):

    ```shell
    sudo ./form-network.sh
    ```

3. Attach the End Device 1 to the new Thread Network:

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
    fdde:ad00:beef:0:0:ff:fe00:1000
    fd49:6624:7914:0:d576:4dbf:8c50:7769
    fdde:ad00:beef:0:3f2b:46dd:ae7d:1282
    fe80:0:0:0:e059:e1a8:724d:3962
    Done
    ```

    Repeat above steps for your Border Router 2 and End Device 2. The only difference is to form the network 2 with `channel 18` rather than `channel 19`.

4. Find the OMR address of each End Device
   1. it is the address that starts with `fd` and it is the single address with its prefix
   2. check the prefix in the Thread network with command `netdata show`
   3. you can also check the logs on each Border Router

5. Ping each End Device from each other:
   
   1. End Device 1 ping End Device 2:
      
      ```shell
      > ping fd29:687e:d8a1:0:c3ef:6492:f0ec:5e8
      Done
      > 16 bytes from fd29:687e:d8a1:0:c3ef:6492:f0ec:5e8: icmp_seq=3 hlim=62 time=56ms
      ```

   2. End Device 2 ping End Device 1:
      
      ```shell
      > ping fd49:6624:7914:0:d576:4dbf:8c50:7769
      Done
      > 16 bytes from fd49:6624:7914:0:d576:4dbf:8c50:7769: icmp_seq=1 hlim=62 time=35ms
      ```

6. You can also ping from Border Router 2 to End Device 1:

    ```shell
    ping -6 fd49:6624:7914:0:d576:4dbf:8c50:7769
    PING fd49:6624:7914:0:d576:4dbf:8c50:7769(fd49:6624:7914:0:d576:4dbf:8c50:7769) 56 data bytes
    64 bytes from fd49:6624:7914:0:d576:4dbf:8c50:7769: icmp_seq=1 ttl=63 time=34.2 ms
    64 bytes from fd49:6624:7914:0:d576:4dbf:8c50:7769: icmp_seq=2 ttl=63 time=23.3 ms
    64 bytes from fd49:6624:7914:0:d576:4dbf:8c50:7769: icmp_seq=3 ttl=63 time=30.8 ms
    ...
    ```
