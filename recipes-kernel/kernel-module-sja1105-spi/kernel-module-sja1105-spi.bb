LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0;md5=801f80980d171dd6425610833a22dbe6"

SRC_URI = " \
	file://Makefile \
	file://switch-spi.c \
	file://sja1105.h \
	file://sja1105_spi.rules \
"

S = "${WORKDIR}"

PV = "${DISTRO_VERSION}"
PR = "r0"

inherit module
EXTRA_OEMAKE += "KERNELDIR=${STAGING_KERNEL_DIR}"
MODULES_INSTALL_TARGET = "install"
KERNEL_MODULE_PACKAGE_SUFFIX = ""

do_install_append() {
	install -D -m 644 ${WORKDIR}/sja1105_spi.rules ${D}${sysconfdir}/udev/rules.d/sja1105_spi.rules
	install -D -m 644 ${WORKDIR}/sja1105.h ${D}/usr/include/linux/sja1105.h
}

FILES_${PN}_append = "${sysconfdir}"