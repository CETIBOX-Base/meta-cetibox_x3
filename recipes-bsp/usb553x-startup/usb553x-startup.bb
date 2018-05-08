SUMMARY = "Script for resettung and enabling the Microchip USB5534B USB hub."
LICENSE = "CLOSED"

inherit systemd 

PV = "${DISTRO_VERSION}"

SRC_URI = " \
		file://usb553x_startup.sh \
		file://usb553x_startup.service \
		file://usb553x_i2c.rules \
"

RDEPENDS_${PN} = "i2c-tools"

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE_${PN} = "usb553x_startup.service"

do_install() {
	install -D -m 744 ${WORKDIR}/usb553x_startup.sh ${D}${bindir}/usb553x_startup.sh
	install -D -m 644 ${WORKDIR}/usb553x_startup.service ${D}${systemd_unitdir}/system/usb553x_startup.service
	install -D -m 644 ${WORKDIR}/usb553x_i2c.rules ${D}${sysconfdir}/udev/rules.d/usb553x_i2c.rules
}
