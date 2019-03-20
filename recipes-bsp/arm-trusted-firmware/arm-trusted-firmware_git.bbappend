FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

ATFW_OPT_append = " ${@base_conditional("CA57CA53BOOT", "1", " PSCI_DISABLE_BIGLITTLE_IN_CA57BOOT=0", "", d)}"
ATFW_OPT_append += " RCAR_DISABLE_NONSECURE_RPC_ACCESS=0"
ATFW_OPT_append += " LIFEC_DBSC_PROTECT_ENABLE=0"

SRC_URI_append = " \
    file://0001-plat-renesas-rcar-Make-RPC-secure-settings-optional.patch \
	file://0002-dont-set-gp2-13-and-gp2-14-for-peripheral-usage.patch \
"

# Use newer version from renesas github to fix DDR/QoS settings issue
PV = "v1.5+renesas+git${SRCPV}"
SRCREV = "11761c9d006eca58b417ef2c1a694caf42c25352"
LIC_FILES_CHKSUM = "file://license.rst;md5=e927e02bca647e14efd87e9e914b2443"

do_compile_append() {
	objcopy -I srec -O binary ${S}/tools/dummy_create/bootparam_sa0.srec ${S}/tools/dummy_create/bootparam_sa0.bin
	objcopy -I srec -O binary ${S}/tools/dummy_create/cert_header_sa6.srec ${S}/tools/dummy_create/cert_header_sa6.bin
}

do_deploy_append() {
	install -m 0644 ${S}/tools/dummy_create/bootparam_sa0.bin ${DEPLOYDIR}/bootparam_sa0.bin
    install -m 0644 ${S}/tools/dummy_create/cert_header_sa6.bin ${DEPLOYDIR}/cert_header_sa6.bin
}
