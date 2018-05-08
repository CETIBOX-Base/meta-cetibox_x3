require u-boot-version.inc

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}/:"
SRC_URI = " \
	file://fw_env.config \
	git://github.com/CETIBOX-Base/u-boot-renesas.git;protocol=https;branch=${BRANCH} \
"

