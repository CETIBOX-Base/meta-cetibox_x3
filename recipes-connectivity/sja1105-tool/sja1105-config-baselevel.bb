SUMMARY = "Default switch configuration files"
SECTION = "connectivity"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
		file://default-h3-gweb-v1.xml \
		file://default-h3-gweb-v1-switch2.xml \
		file://default-h3-vc2-v3.xml \
		file://default-h3-vc2-v3-switch2.xml \
"

RDEPENDS_${PN} = "sja1105-tool"

do_compile() {
}

do_install() {
	install -D -m 644 ${WORKDIR}/default-h3-gweb-v1.xml ${D}/etc/sja1105/default-h3-gweb-v1.xml
	install -D -m 644 ${WORKDIR}/default-h3-gweb-v1-switch2.xml ${D}/etc/sja1105/default-h3-gweb-v1-switch2.xml
	install -D -m 644 ${WORKDIR}/default-h3-vc2-v3.xml ${D}/etc/sja1105/default-h3-vc2-v3.xml
	install -D -m 644 ${WORKDIR}/default-h3-vc2-v3-switch2.xml ${D}/etc/sja1105/default-h3-vc2-v3-switch2.xml
}
