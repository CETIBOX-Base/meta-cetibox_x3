FILESEXTRAPATHS_prepend := "${THISDIR}/files/:"

SRC_URI_append = " \
	file://0001-Add-fb-close-Use-dev-tty0-current-VT-for-open.patch \
"

do_install_append() {
	install -m 0755 fb-close ${D}${bindir}/fb-close
}
