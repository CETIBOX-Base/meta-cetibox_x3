LICENSE = "GPLv2"

LIC_FILES_CHKSUM = "file://COPYING;md5=d7810fab7487fb0aad327b76f1be7cd7"

COMPATIBLE_MACHINE = "cetibox-m3ulcb|cetibox-h3ulcb"

do_configure[depends] += "virtual/kernel:do_shared_workdir"

inherit linux-kernel-base kernel-arch

do_populate_lic[depends] += "virtual/kernel:do_patch"

do_compile() {
}

do_install[depends] += "virtual/kernel:do_compile"

inherit kernelsrc

do_install() {
	oe_runmake headers_install INSTALL_HDR_PATH=${D}${includedir}/linux-cetibox
	mv ${D}${includedir}/linux-cetibox/include/* ${D}${includedir}/linux-cetibox
	rmdir ${D}${includedir}/linux-cetibox/include
}

PACKAGES = "${PN}"
FILES_${PN} = "${includedir}/linux-cetibox/*"

LINUX_VERSION ?= "4.14.75"
PV = "${LINUX_VERSION}"
PR = "r1"
