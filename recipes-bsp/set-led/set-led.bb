SUMMARY = "Script for setting the front panel leds to different values."
LICENSE = "CLOSED"

inherit systemd

PV = "${DISTRO_VERSION}"

SRC_URI = " \
		file://set_led \
		file://power_led.service \
		file://heartbeat_led \
		file://heartbeat_led.service \
"

RDEPENDS_${PN} = "i2c-tools"

SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE_${PN} = "power_led.service heartbeat_led.service"

do_install() {
	install -D -m 744 ${WORKDIR}/set_led ${D}${bindir}/set_led

	install -D -m 644 ${WORKDIR}/power_led.service ${D}${systemd_unitdir}/system/power_led.service

	install -D -m 744 ${WORKDIR}/heartbeat_led ${D}${bindir}/heartbeat_led
	install -D -m 644 ${WORKDIR}/heartbeat_led.service ${D}${systemd_unitdir}/system/heartbeat_led.service
}
