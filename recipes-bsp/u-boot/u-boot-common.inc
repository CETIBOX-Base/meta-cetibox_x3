HOMEPAGE = "http://www.denx.de/wiki/U-Boot/WebHome"
SECTION = "bootloaders"

LICENSE = "GPLv2+"
LIC_FILES_CHKSUM = "file://Licenses/README;md5=a2c678cfd4a4d97135585cad908541c6"
PE = "1"

SRCREV = "e6805f07405e83c8242c03af921288f8420002b7"
BRANCH = "v2018.05/rcar-ctc"
PV = "2018.05+rcar+ctc"

SRC_URI = " \
	git://github.com/CETIBOX-Base/u-boot-renesas.git;protocol=https;branch=${BRANCH} \
"

S = "${WORKDIR}/git"
