LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
	file://Ethernet \
	file://NetworkManager.conf \
"
do_compile() {
}

do_install() {
	install -D -m 600 ${WORKDIR}/Ethernet ${D}${sysconfdir}/NetworkManager/system-connections/Ethernet
	install -D -m 600 ${WORKDIR}/NetworkManager.conf ${D}${sysconfdir}/NetworkManager/NetworkManager.conf
}
