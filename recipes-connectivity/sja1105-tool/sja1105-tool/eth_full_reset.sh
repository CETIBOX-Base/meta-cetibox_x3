#!/bin/sh
GPIO_PATH=/sys/class/gpio
#GPIO_462_PATH="$GPIO_PATH"/gpio462

BOARD=$(boardid)

echo 399 > "$GPIO_PATH"/export 2>/dev/null || echo "GPIO399 already exported"
echo 497 > "$GPIO_PATH"/export 2>/dev/null || echo "GPIO497 already exported"
echo 498 > "$GPIO_PATH"/export 2>/dev/null || echo "GPIO498 already exported"
echo out > "$GPIO_PATH"/gpio399/direction || exit 1
echo out > "$GPIO_PATH"/gpio497/direction || exit 1
echo out > "$GPIO_PATH"/gpio498/direction || exit 1

# Reset all PHYs
#echo 462 > "$GPIO_PATH"/export
#echo out > "$GPIO_462_PATH"/direction
#echo 0   > "$GPIO_462_PATH"/value
#sleep 0.1
#echo 1   > "$GPIO_462_PATH"/value
#echo in  > "$GPIO_462_PATH"/direction
#echo 462 > "$GPIO_PATH"/unexport

#sleep 0.1

if [ -f /etc/default/sja1105 ]; then
	. /etc/default/sja1105
fi

if [ -z "${BR2_ETH_MUX+x}" ]; then
	BR2_ETH_MUX=ETH
fi

if [ -z "${BR1_MASTER_MODE}" ]; then
	BR1_MASTER_MODE=SLAVE
fi

if [ -z "${BR2_MASTER_MODE}" ]; then
	BR2_MASTER_MODE=SLAVE
fi

if [ -z "${BR3_MASTER_MODE}" ]; then
	BR3_MASTER_MODE=SLAVE
fi

if [ -z "${BR4_MASTER_MODE}" ]; then
	BR4_MASTER_MODE=SLAVE
fi

if [ -z "${BR5_MASTER_MODE}" ]; then
	BR5_MASTER_MODE=SLAVE
fi

set_phy_master() {
	BR=$1
	MODE=$2

	case "$BOARD" in
		"h3-gweb-v1")
			case "$BR" in
				1)
					echo 0 > /sys/class/gpio/gpio497/value
					echo 1 > /sys/class/gpio/gpio498/value
					PHYID=4
					;;
				2)
					echo 1 > /sys/class/gpio/gpio497/value
					echo 0 > /sys/class/gpio/gpio498/value
					PHYID=4
					;;
				3|4|5)
					PHYID=$((BR+2))
					;;
				*)
					echo "Invalid BroadR-Reach $BR"
					exit 1
					;;
			esac
			# Register access enable
			phytool write eth0/$PHYID/17 0x9806
			if [ "$MODE" = "MASTER" ]; then
				# Set master mode
				phytool write eth0/$PHYID/18 0xd811
			else
				phytool write eth0/$PHYID/18 0x5811
			fi
			# Reset
			phytool write eth0/$PHYID/0 0xa100
			;;
		"h3-vc2-v3")
			case "$BR" in
				1)
					echo 0 > /sys/class/gpio/gpio497/value
					echo 1 > /sys/class/gpio/gpio498/value
					PHYID=0
					;;
				2)
					echo 1 > /sys/class/gpio/gpio497/value
					echo 0 > /sys/class/gpio/gpio498/value
					PHYID=0
					;;
				3|4|5)
					PHYID=$((BR-2))
					;;
				*)
					echo "Invalid BroadR-Reach $BR"
					exit 1
					;;
			esac
			phytool write eth0/$PHYID/0x1f 0x0000
			if [ "$MODE" = "MASTER" ]; then
				phytool write eth0/$PHYID/0x9  0x0800
			else
				phytool write eth0/$PHYID/0x9  0x0000
			fi
			# Reset
			phytool write eth0/$PHYID/0 0xa100
			;;
	esac
}

case "$BOARD" in
	"h3-gweb-v1")
		# interface must be up for phytool to access PHYs
		ip link set dev eth0 up

		# No SJA1105 reset, CLPD in the way

		# Set 0.96 ns TXCLK delay on RGMII 10/100/1000 PHY
		phytool write eth0/2/0xd 0x0002 || exit 1
		phytool write eth0/2/0xe 0x0008 || exit 1
		phytool write eth0/2/0xd 0x4002 || exit 1
		phytool write eth0/2/0xe 0x03ef || exit 1

		if [ "$BR2_ETH_MUX" = "ETH" ]; then
			# Switch BR2/second ETH multiplexer to ETH
			echo 1 > "$GPIO_PATH"/gpio399/value || exit 1

			# Enable 10/00 on second ethernet PHY
			phytool write eth0/1/4 0x01e1 || exit 1
			phytool write eth0/1/0 0x1200 || exit 1
		else
			# Switch BR2/second ETH multiplexer to BR2
			echo 0 > "$GPIO_PATH"/gpio399/value || exit 1
		fi
		;;
	"h3-vc2-v3")
		# interface must be up for phytool to access PHYs
		ip link set dev eth0 up

		# Deassert switch reset (TODO FIXME race condition)
		i2cset -y 5 0x3c 0 0xe

		# Configure KSZ8091RNB phy to RMII mode by setting "Operation Mode Strap Override" register
		# Necessary due to incorrect value of "Operation Mode Strap Status" after reset (why?) which
		# puts phy in MII(?) mode.
		# Also disable the broadcast on KSZ8091 (bit9)
		phytool write eth0/5/0x16 0x0202

		# Disable the broadcast on RTL9000 (bit13)
		# BR1
		echo 0 > /sys/class/gpio/gpio497/value
		echo 1 > /sys/class/gpio/gpio498/value
		phytool write eth0/0/0x31 0x0a43
		phytool write eth0/0/0x18 0x0098

		# BR2
		echo 1 > /sys/class/gpio/gpio497/value
		echo 0 > /sys/class/gpio/gpio498/value
		phytool write eth0/0/0x31 0x0a43
		phytool write eth0/0/0x18 0x0098

		# BR3,4,5
		for i in 1 2 3; do
			phytool write eth0/$i/0x31 0x0a43
			phytool write eth0/$i/0x18 0x0098
		done

		if [ "$BR2_ETH_MUX" = "ETH" ]; then
			# Switch BR2/second ETH multiplexer to ETH
			echo 1 > "$GPIO_PATH"/gpio399/value || exit 1
		else
			# Switch BR2/second ETH multiplexer to BR2
			echo 0 > "$GPIO_PATH"/gpio399/value || exit 1
		fi
		;;
	*)
		echo "Cannot reset ethernet, unknown board $BOARD"
		exit 1
		;;
esac

set_phy_master 1 $BR1_MASTER_MODE
if [ "$BR2_ETH_MUX" = "BR2" ]; then
	set_phy_master 2 $BR2_MASTER_MODE
fi
set_phy_master 3 $BR3_MASTER_MODE
set_phy_master 4 $BR4_MASTER_MODE
set_phy_master 5 $BR5_MASTER_MODE
