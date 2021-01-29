SUMMARY = "Script to read the board variant from the device tree's compatible string"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://COPYING;md5=bae71f312b6416ca38aec27d0ecf9237"

inherit allarch

SRC_URI = " \
	file://boardid \
	file://COPYING \
"

S = "${WORKDIR}"

do_install_append() {
	install -D -m 755 ${WORKDIR}/boardid ${D}${bindir}/boardid
}
