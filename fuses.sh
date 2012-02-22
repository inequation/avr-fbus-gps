#!/bin/sh
avrdude -p t2313 -c usbasp -U hfuse:w:0x9B:m -U lfuse:w:0xFD:m
