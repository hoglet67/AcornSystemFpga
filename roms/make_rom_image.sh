#!/bin/bash

# Create the 64K ROM image
#
# This contains:
# 0x8000-0xFFFF (System 3)
# 0x8000-0xFFFF (System 5)


IMAGE=tmp/rom_image.bin

rm -f $IMAGE

# System 3 ROM Images

cat generic/blank.rom          >> $IMAGE
cat generic/blank.rom          >> $IMAGE
cat generic/SBASIC1.rom        >> $IMAGE
cat generic/blank.rom          >> $IMAGE
cat system3/TOSDOS-S3.rom      >> $IMAGE

# System 5 ROM Images

cat generic/blank.rom          >> $IMAGE
cat generic/blank.rom          >> $IMAGE
cat generic/SBASIC1.rom        >> $IMAGE
cat generic/blank.rom          >> $IMAGE
cat system5/System5-1F.rom     >> $IMAGE
