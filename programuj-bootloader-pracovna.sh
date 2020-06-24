#!/bin/bash
date
mosquitto_pub -t "/thermctl-in/PRACOVNA/bootloader" -m "1" -h 192.168.1.120
sleep 2

ls -la  room-control-eth.ino_atmega1284p_16000000L.hex
avr-objcopy -I ihex room-control-eth.ino_atmega1284p_16000000L.hex -O binary room-control-eth.ino_atmega1284p_16000000L.bin
~/elektronika/saric-bootloader/software/main -f room-control-eth.ino_atmega1284p_16000000L.bin -d 192.168.2.112
#sudo avrdude -c usbasp -p m1284 -V  -U flash:w:term-big-eth.ino_atmega1284_16000000L.hex:i
#rm  term-big-eth.ino_atmega1284_16000000L.hex term-big-eth.ino_atmega1284_16000000L.bin
