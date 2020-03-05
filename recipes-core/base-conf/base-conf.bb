LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
DESCRIPTION = "Misc. basic configuration files for CETiBox"

SRC_URI = " \
	file://locale.conf \
"

do_install() {
	install -D -m 644 ${WORKDIR}/locale.conf ${D}${sysconfdir}/locale.conf
}
