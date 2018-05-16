LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
	file://persistent-can-if-naming.rules \
	file://spidev-systemd.rules \
"

do_install() {
	install -D -m 644 ${WORKDIR}/persistent-can-if-naming.rules ${D}${sysconfdir}/udev/rules.d/persistent-can-if-naming.rules
	install -D -m 644 ${WORKDIR}/spidev-systemd.rules ${D}${sysconfdir}/udev/rules.d/spidev-systemd.rules
}
