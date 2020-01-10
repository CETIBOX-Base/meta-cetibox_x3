HOMEPAGE = "http://www.denx.de/wiki/U-Boot/WebHome"
SECTION = "bootloaders"

LICENSE = "GPLv2+"
LIC_FILES_CHKSUM = "file://Licenses/README;md5=a2c678cfd4a4d97135585cad908541c6"
PE = "1"

SRCREV = "bc446f84ac94ac2c56206e162129697eb0fc037a"
BRANCH = "vc2-support"
PV = "2018.05+rcar+ctc"

SRC_URI = " \
	git://github.com/CETIBOX-Base/u-boot-renesas.git;protocol=https;branch=${BRANCH} \
"

S = "${WORKDIR}/git"
