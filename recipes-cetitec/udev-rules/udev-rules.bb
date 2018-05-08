LICENSE = "CLOSED"

SRC_URI_append = "file://persistent-can-if-naming.rules"

do_install_append() {
	install -D -m 644 ${WORKDIR}/persistent-can-if-naming.rules ${D}${sysconfdir}/udev/rules.d/persistent-can-if-naming.rules
}

#FILES_${PN}_append = "persistent-can-if-naming.rules"
