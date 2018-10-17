# Copyright (C) 2018 Stefan Kratochwil <stefan.kratochwil@cetitec.com>
# Released under the MIT license (see COPYING.MIT for the terms)

DESCRIPTION = "Installs the evaluation version of the CETiTEC Universal Gateway."
HOMEPAGE = "http://www.cetitec.com"
LICENSE = "CLOSED"
SECTION = ""
DEPENDS = " \
    libsocketcan \
"

SRC_URI = " \
	file://UgwEval.tar \
"

inherit bin_package

do_install() {
    install -D -m 0755 ${WORKDIR}/UgwEval ${D}${bindir}/UgwEval
}
