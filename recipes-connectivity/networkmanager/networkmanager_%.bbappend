DEPENDS_remove = "polkit"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}/:"
SRC_URI_append = " \
	file://Ethernet \
	file://NetworkManager.conf \
"

PACKAGECONFIG[systemd] = " \
	--with-systemdsystemunitdir=${systemd_unitdir}/system --with-session-tracking=systemd --enable-polkit=disabled, \
	--without-systemdsystemunitdir \
"

FILES_${PN} += " \
	${sysconfdir}/NetworkManager/system-connections/ \
"

FILES_${PN}-dev += "${libdir}/girepository-1.0"

do_install_append() {
	install -D -m 600 ${WORKDIR}/Ethernet ${D}${sysconfdir}/NetworkManager/system-connections/Ethernet
	install -D -m 600 ${WORKDIR}/NetworkManager.conf ${D}${sysconfdir}/NetworkManager/NetworkManager.conf
}

