DEPENDS_remove = "polkit"

PACKAGECONFIG[systemd] = " \
	--with-systemdsystemunitdir=${systemd_unitdir}/system --with-session-tracking=systemd --enable-polkit=disabled, \
	--without-systemdsystemunitdir \
"

FILES_${PN}-dev += "${libdir}/girepository-1.0"

