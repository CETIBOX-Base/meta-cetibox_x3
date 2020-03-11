LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
DESCRIPTION = "Misc. basic configuration files for CETiBox"

SRC_URI = " \
	file://locale.conf \
	file://set_lang.sh \
"

do_install() {
	install -D -m 644 ${WORKDIR}/locale.conf ${D}${sysconfdir}/locale.conf
	install -D -m 644 ${WORKDIR}/set_lang.sh ${D}${sysconfdir}/profile.d/set_lang.sh
}

# Create groups at build time otherwise created by sysusers.d at first
# boot to avoid /etc/groups being overwritten in overlayfs
inherit useradd
USERADD_PACKAGES = "${PN}"
GROUPADD_PARAM_${PN} = "-r nobody; -r render; -r wheel;"
