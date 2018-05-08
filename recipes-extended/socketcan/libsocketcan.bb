SUMMARY = "Control basic functions in socketcan from userspace"
HOMEPAGE = "http://www.pengutronix.de"
SECTION = "libs/network"

LICENSE = "LGPLv2.1"
LIC_FILES_CHKSUM = "file://src/libsocketcan.c;beginline=3;endline=18;md5=c5021d7184d2e7487bb954ccf2f4f30c"

SRC_URI = " \
	git://github.com/CETIBOX-Base/libsocketcan.git;protocol=https \
"

SRCREV = "e9d1b6ec8b449e6ac3f0809761b62a242f1b9be9"
PV = "0.0.10+ctc"

S = "${WORKDIR}/git"
inherit autotools pkgconfig

do_configure_prepend() {
    sed -i -e s:tests/GNUmakefile::g -e s:trunk:0.0.10: ${S}/configure.ac
}
