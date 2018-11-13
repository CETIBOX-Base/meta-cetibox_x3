# Copyright (C) 2018 Stefan Kratochwil <stefan.kratochwil@cetitec.com>
# Released under the MIT license (see COPYING.MIT for the terms)

DESCRIPTION = "Installs the evaluation version of the CETiTEC ACS suite."
HOMEPAGE = "http://www.cetitec.com"
LICENSE = "CLOSED"

SRC_URI = " \
	file://acsdaemon-eval.tar;subdir=${PN}-${PV} \
	file://acslib-eval.tar;subdir=${PN}-${PV} \
	file://testfblock-eval.tar;subdir=${PN}-${PV} \
	file://libk2lmostdriver-eval.tar;subdir=${PN}-${PV} \
"

inherit bin_package

INSANE_SKIP_${PN} += " already-stripped"

FILES_${PN}-dev = ""
FILES_${PN} = " \
    ${libdir}/libacsplatform.so \
    ${libdir}/libacsbase.so \
    ${libdir}/libacsclient.so \
    ${libdir}/libacsevents.so \
    ${libdir}/libacstcplib.so \
    ${libdir}/libacsmost.so \
    ${libdir}/libacstinymost.so \
    ${libdir}/libacsdebug.so \
"

do_install() {
    install -d -m0755 ${D}${libdir}
    install -m 0755 usr/lib/libacsplatform.so ${D}${libdir}/libacsplatform.so
    install -m 0755 usr/lib/libacsbase.so ${D}${libdir}/libacsbase.so
    install -m 0755 usr/lib/libacsclient.so ${D}${libdir}/libacsclient.so
    install -m 0755 usr/lib/libacsevents.so ${D}${libdir}/libacsevents.so
    install -m 0755 usr/lib/libacstcplib.so ${D}${libdir}/libacstcplib.so
    install -m 0755 usr/lib/libacsmost.so ${D}${libdir}/libacsmost.so
    install -m 0755 usr/lib/libacstinymost.so ${D}${libdir}/libacstinymost.so
    install -m 0755 usr/lib/libacsdebug.so ${D}${libdir}/libacsdebug.so
}
