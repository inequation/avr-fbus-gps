#!/bin/sh
avrdude -p t2313 -c usbasp -U flash:w:gps.hex:i -U eeprom:w:gps.eep:i
