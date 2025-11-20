#!/bin/bash

FIRMWARE=$1
if [ -z "${FIRMWARE}" ]; then
  echo "Usage: $(basename $0) firmware"
  exit 1
fi

TYPE=elf
if [[ "${FIRMWARE}" == *.elf ]]; then
  TYPE=elf
elif [[ "${FIRMWARE}" == *.bin ]]; then
  TYPE=bin
elif [[ "${FIRMWARE}" == *.hex ]]; then
  TYPE=ihex
fi

gpioset $(gpiofind enable_1v8)=1

declare -A PINS
# swclk,swdio,srst
PINS[x]="37 34 118"
PINS[y]="68 66 43"
PINS[z]="59 58 38"

for axis in x y z
do
  read SWCLK SWDIO SRST <<<"${PINS[$axis]}"
  openocd                                                                      \
    -c "adapter driver linuxgpiod"                                             \
    -c "reset_config srst_only srst_nogate connect_assert_srst srst_push_pull" \
    -c "transport select swd"                                                  \
    -c "set CHIPNAME saml21"                                                   \
    -c "source [find target/at91samdXX.cfg]"                                   \
    -c "adapter gpio swclk $SWCLK -chip 0"                                     \
    -c "adapter gpio swdio $SWDIO -chip 0"                                     \
    -c "adapter gpio srst $SRST -chip 0 -active-high"                          \
    -c "tcl_port 0"                                                            \
    -c "telnet_port 0"                                                         \
    -c "gdb_port 0"                                                            \
    -c "init"                                                                  \
    -c "targets"                                                               \
    -c "reset halt"                                                            \
    -c "flash write_image erase \"${FIRMWARE}\" 0 ${TYPE}"                     \
    -c "verify_image \"${FIRMWARE}\" 0"                                        \
    -c "reset run"                                                             \
    -c "exit" 1> >(sed "s/^/[$axis] /") 2> >(sed "s/^/[$axis] /" >&2) &
done
wait
