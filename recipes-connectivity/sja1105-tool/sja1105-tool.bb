SUMMARY = "Linux userspace application for configuring the NXP SJA1105 Automotive Ethernet L2 switch"
SECTION = "connectivity"
LICENSE = "BSD-3-Clause"
LIC_FILES_CHKSUM = "file://COPYING;md5=bac7a51138672c261de0ab4017b7c396"

inherit pkgconfig systemd

PV = "1.1-ctc"
S = "${WORKDIR}/git"

SRC_URI = " \
	git://github.com/CETIBOX-Base/sja1105-tool.git;protocol=https;rev=${SRCREV} \
	file://eth_full_reset.sh \
	file://sja1105-2.conf \
	file://sja1105-switch2.service \
	file://sja1105.conf \
	file://sja1105.service \
"

SRCREV = "e14a6bb6030733d4b974cdc5142a52d389510498"

DEPENDS = "libxml2 kernel-module-sja1105-spi phytool boardid"

RDEPENDS_${PN}_append = " kernel-module-sja1105-spi phytool boardid"

CFLAGS_append = " -Wno-error=array-bounds"

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE_${PN} = " \
	sja1105.service \
	sja1105-switch2.service \
"

# use unversioned shared object
SOLIBS = ".so"
FILES_SOLIBSDEV = ""

do_install() {
	install -D -m 755 ${S}/libsja1105.so ${D}${libdir}/libsja1105.so
	install -D -m 755 ${S}/sja1105-tool ${D}${bindir}/sja1105-tool
	install -D -m 744 ${WORKDIR}/eth_full_reset.sh ${D}${bindir}/eth_full_reset.sh
	install -D -m 644 ${WORKDIR}/sja1105.conf ${D}/etc/sja1105/sja1105.conf
	install -D -m 644 ${WORKDIR}/sja1105.service ${D}${systemd_unitdir}/system/sja1105.service
	install -D -m 644 ${WORKDIR}/sja1105-2.conf ${D}/etc/sja1105/sja1105-2.conf
	install -D -m 644 ${WORKDIR}/sja1105-switch2.service ${D}${systemd_unitdir}/system/sja1105-switch2.service
}

FILES_${PN} = "${libdir}/libsja1105.so ${bindir}/sja1105-tool ${systemd_unitdir}/system /etc ${bindir}/eth_full_reset.sh"
