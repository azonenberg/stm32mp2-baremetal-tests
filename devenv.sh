#!/bin/sh

# Relaunch OpenOCD stuff
screen -d -m -S mp2-openocd-server /ceph/fast/home/azonenberg/code/3rdparty/openocd/src/openocd -f openocd-orb.cfg
sleep 1
screen -d -m -S mp2-openocd-client telnet localhost 4901

# UARTs are not stable, so need to reconnect that manually :(

# Build dir
cd /ceph/fast/home/azonenberg/code/stm32mp2-baremetal-tests/src/cpu2-m33/debug-build
screen -d -m -S mp2-m33-build

# GDB stuff
cd /ceph/fast/home/azonenberg/code/stm32mp2-baremetal-tests/src/cpu2-m33
screen -d -m -S mp2-m33-gdb

cd /ceph/fast/home/azonenberg/code/stm32mp2-baremetal-tests/src/cpu1-a35
screen -d -m -S mp2-a35-gdb
