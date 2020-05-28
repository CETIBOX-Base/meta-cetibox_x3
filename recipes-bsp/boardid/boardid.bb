SUMMARY = "Script to read the board variant from the device tree's compatible string"
LICENSE = "CLOSED"

inherit allarch

SRC_URI = "file://boardid"

do_install_append() {
	install -D -m 755 ${WORKDIR}/boardid ${D}${bindir}/boardid
}
