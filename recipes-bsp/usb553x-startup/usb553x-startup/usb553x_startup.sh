#!/bin/sh

# Reset USB hub via CPLD
echo "FIXME: Do proper read/modify/update!"
# The mostcore driver crashes the kernel when resetting the MOST INIC
# during bootup, which is why we don't write a 0x00 to register 0x00
# anymore (bit 3 is responsible for resetting the INIC).
# However, we need to implement this access as proper read/modify/update
# modification. This is considered future work.
i2cset -y 5 0x3c 0x00 0x04
i2cset -y 5 0x3c 0x00 0x06

# Put USB hub operational, leave I2C slave active
i2cset -y 5 0x2d 0xaa 0x56 0x00 i
