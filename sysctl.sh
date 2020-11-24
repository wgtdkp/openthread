#!/bin/bash

sudo sysctl -w net.ipv6.conf.all.forwarding=1 
sudo sysctl -w net.ipv6.conf.wlan0.accept_ra=2
sudo sysctl -w net.ipv6.conf.wlan0.accept_ra_rt_info_max_plen=128
