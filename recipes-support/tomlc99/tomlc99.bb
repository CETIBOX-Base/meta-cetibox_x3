# Copyright (C) 2019 CETiTEC
DESCRIPTION = "TOML in c99; v0.5.0 compliant."
HOMEPAGE = "https://github.com/cktan/tomlc99"
LICENSE = "MIT"
SECTION = "support"
LIC_FILES_CHKSUM = "file://LICENSE;md5=815f47e0a2e8e641301a9f6de604ccdc"

SRC_URI = "git://github.com/cktan/tomlc99.git"
SRCREV = "f31bcd0adfecf8a0817c0423f3e746f320434182"
PV = "git${SRCPV}"

S = "${WORKDIR}/git"
TARGET_CC_ARCH += "${LDFLAGS}"

do_compile() {
	oe_runmake
}

do_install() {
	oe_runmake prefix="${D}/usr" install
}

SOLIBSDEV = ""
SOLIBS = ".so"

FILES_${PN}-dev = "usr/include"
#FILES_${PN} += "usr/lib/libtoml.so"
