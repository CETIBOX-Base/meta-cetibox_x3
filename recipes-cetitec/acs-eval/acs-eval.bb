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
inherit systemd

INSANE_SKIP_${PN} += " already-stripped"

FILES_${PN}-dev = ""
FILES_ACSLIB_EVAL = " \
    ${libdir}/libacsplatform.so \
    ${libdir}/libacsbase.so \
    ${libdir}/libacsclient.so \
    ${libdir}/libacsevents.so \
    ${libdir}/libacstcplib.so \
    ${libdir}/libacsmost.so \
    ${libdir}/libacstinymost.so \
    ${libdir}/libacsdebug.so \
"

FILES_ACSDAEMON_EVAL = " \
    ${bindir}/acsdaemon-eval \
    ${systemd_unitdir}/system/acsdaemon-eval.service \
    ${sysconfdir}/acsdaemon.cfg \
"

FILES_LIBK2LMOSTDRIVER_EVAL = " \
    ${libdir}/libk2lmostdriver-eval.so \
"

FILES_TESTFBLOCK_EVAL = " \
    ${bindir}/TestFBlock-eval \
    ${systemd_unitdir}/system/testfblock-eval@.service \
"

FILES_${PN} = " \
    ${FILES_ACSLIB_EVAL} \
    ${FILES_ACSDAEMON_EVAL} \
    ${FILES_LIBK2LMOSTDRIVER_EVAL} \
    ${FILES_TESTFBLOCK_EVAL} \
"

SYSTEMD_SERVICE = " \
    acsdaemon-eval.service \
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

    install -d -m0755 ${D}${bindir}
    install -D -m 755 usr/bin/acsdaemon-eval ${D}${bindir}/acsdaemon-eval
    install -d -m0755 ${D}${systemd_unitdir}
    install -D -m 644 lib/systemd/system/acsdaemon-eval.service ${D}${systemd_unitdir}/system/acsdaemon-eval.service
    install -d -m0755 ${D}${sysconfdir}
    install -D -m 644 etc/acsdaemon.cfg ${D}${sysconfdir}/acsdaemon.cfg

    install -m 0755 usr/lib/libk2lmostdriver-eval.so ${D}${libdir}/libk2lmostdriver-eval.so
    
    install -D -m 755 usr/bin/TestFBlock-eval ${D}${bindir}/TestFBlock-eval
    install -D -m 644 lib/systemd/system/testfblock-eval@.service ${D}${systemd_unitdir}/system/testfblock-eval@.service
}
