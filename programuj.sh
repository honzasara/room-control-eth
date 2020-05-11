#!/bin/bash
date
ls -la room-control-eth.ino_atmega1284_16000000L.hex && sudo avrdude -c usbasp -p m1284 -V  -U flash:w:room-control-eth.ino_atmega1284_16000000L.hex:i && rm room-control-eth.ino_atmega1284_16000000L.hex
