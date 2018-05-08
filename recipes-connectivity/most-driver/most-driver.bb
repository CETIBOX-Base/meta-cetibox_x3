LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://mostcore/core.c;beginline=4;endline=11;md5=ee925010da4c7088ce5d63356671c634"

SRC_URI = "git://github.com/CETIBOX-Base/most-driver.git;protocol=https;branch=rcar3 \
"

S = "${WORKDIR}/git"
SRCREV = "9e8ea9fc6fc78802a2a7513aba95f1ae0f7e4331"

inherit module

EXTRA_OEMAKE += "CONFIG_HDM_USB=m M=${S} KDIR=${STAGING_KERNEL_DIR}"

