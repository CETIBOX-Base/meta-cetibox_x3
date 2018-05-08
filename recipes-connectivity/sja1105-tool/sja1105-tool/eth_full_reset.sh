#!/bin/sh
GPIO_PATH=/sys/class/gpio

# interface must be up for phytool to access PHYs
ip link set dev eth0 up

# No SJA1105 reset, CLPD in the way

# Set 0.96 ns TXCLK delay on RGMII 10/100/1000 PHY
phytool write eth0/2/0xd 0x0002 || exit 1
phytool write eth0/2/0xe 0x0008 || exit 1
phytool write eth0/2/0xd 0x4002 || exit 1
phytool write eth0/2/0xe 0x03ef || exit 1

# Switch BR2/second ETH multiplexer to ETH
echo 399 > "$GPIO_PATH"/export 2>/dev/null || echo "GPIO399 already exported"
echo out > "$GPIO_PATH"/gpio399/direction || exit 1
echo 1 > "$GPIO_PATH"/gpio399/value || exit 1
# Enable 10/00 on second ethernet PHY
phytool write eth0/1/4 0x01e1 || exit 1
phytool write eth0/1/0 0x1200 || exit 1
