#!/usr/bin/env bash

# set -x
# ELF=build/stm32/core-bms-f34.elf
# openocd -f ./openocd.cfg -c "program ${ELF} verify reset" -c "exit"

# Load the project name from the Makefile
PROJECT_NAME=template
PROJECT_VERSION=f33
if [[ $(cat Makefile) =~ PROJECT_NAME[[:space:]]*:=[[:space:]]*([^[:space:]]*) ]]; then 
    PROJECT_NAME=${BASH_REMATCH[1]}
fi
if [[ $(cat Makefile) =~ PROJECT_VERSION[[:space:]]*:=[[:space:]]*([^[:space:]]*) ]]; then 
    PROJECT_VERSION=${BASH_REMATCH[1]}
fi

ELF=build/stm32/core-vc-f34.elf

echo "Programming with $ELF"

mkfifo .uploader_telnet_fifo
echo -e "program ${ELF} verify reset\nexit" > /tmp/uploader_telnet_fifo &
tail -f /tmp/uploader_telnet_fifo | telnet localhost 4444
if [ $? != 0 ]; then
    openocd -f ./openocd.cfg -c "program ${ELF} verify reset" -c "exit"
fi
rm .uploader_telnet_fifo
