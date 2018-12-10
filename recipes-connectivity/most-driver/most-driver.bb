LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://mostcore/core.c;beginline=4;endline=11;md5=ee925010da4c7088ce5d63356671c634"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}/:"
SRC_URI = " \
    git://github.com/CETIBOX-Base/most-driver.git;protocol=https;branch=rcar3 \
    file://most-driver.rules \
    file://most-setup \
    file://most-driver-setup.service \
    file://most-driver-setup-channels.sh \
"

FILES_${PN}_append = " \
    /lib/udev/most-setup \
    /etc/udev/rules.d/most-driver.rules \
    ${bindir}/most-driver-setup-channels.sh \
    ${systemd_unitdir}/system/most-driver-setup.service \
"

S = "${WORKDIR}/git"
SRCREV = "9e8ea9fc6fc78802a2a7513aba95f1ae0f7e4331"

do_install_append() {
    install -D -m 644 ${WORKDIR}/most-driver.rules ${D}${sysconfdir}/udev/rules.d/most-driver.rules
    install -D ${WORKDIR}/most-setup ${D}/lib/udev/most-setup

    install -D -m 644 ${WORKDIR}/most-driver-setup.service ${D}${systemd_unitdir}/system/most-driver-setup.service
    install -D -m 744 ${WORKDIR}/most-driver-setup-channels.sh ${D}${bindir}/most-driver-setup-channels.sh
}

inherit module

EXTRA_OEMAKE += "CONFIG_HDM_USB=m M=${S} KDIR=${STAGING_KERNEL_DIR}"

inherit systemd
SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE_${PN} = " \
    most-driver-setup.service \
"
