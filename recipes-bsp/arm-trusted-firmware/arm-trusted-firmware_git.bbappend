FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

COMPATIBLE_MACHINE =. "|cetibox-h3"

ATFW_OPT_append = " ${@base_conditional("CA57CA53BOOT", "1", " PSCI_DISABLE_BIGLITTLE_IN_CA57BOOT=0", "", d)}"
ATFW_OPT_append += " RCAR_DISABLE_NONSECURE_RPC_ACCESS=0"
ATFW_OPT_append += " LIFEC_DBSC_PROTECT_ENABLE=0"

SRC_URI_append = " \
    file://0001-plat-renesas-rcar-Make-RPC-secure-settings-optional.patch \
	file://0002-dont-set-gp2-13-and-gp2-14-for-peripheral-usage.patch \
"

# Build firmware for CETiBox with H3ULCB
ATFW_OPT_append_cetibox-h3 = " RCAR_GEN3_ULCB=1 PMIC_LEVEL_MODE=0"

# More magic
python do_extra_ipl_opt_append() {
    if soc == "r8a7795":
        # Build firmware for CETiBox VC2 with integrated H3 SOP
        d.setVar('EXTRA_ATFW_CONF', 'vc2')
        d.setVar('EXTRA_ATFW_OPT', ' LSI=H3 RCAR_DRAM_SPLIT=1 RCAR_DRAM_LPDDR4_MEMCONF=0 ${ATFW_OPT_LOSSY} RCAR_SYSTEM_SUSPEND=0')
        bb.build.exec_func('do_ipl_opt_compile', d)
        bb.build.exec_func('do_ipl_opt_deploy', d)
}

do_compile_append() {
	objcopy -I srec -O binary ${S}/tools/dummy_create/bootparam_sa0.srec ${S}/tools/dummy_create/bootparam_sa0.bin
	objcopy -I srec -O binary ${S}/tools/dummy_create/cert_header_sa6.srec ${S}/tools/dummy_create/cert_header_sa6.bin
}

do_deploy_append() {
	install -m 0644 ${S}/tools/dummy_create/bootparam_sa0.bin ${DEPLOYDIR}/bootparam_sa0.bin
    install -m 0644 ${S}/tools/dummy_create/cert_header_sa6.bin ${DEPLOYDIR}/cert_header_sa6.bin
}

do_ipl_opt_compile_append() {
	objcopy -I srec -O binary ${S}/tools/dummy_create/bootparam_sa0.srec ${S}/tools/dummy_create/bootparam_sa0.bin
	objcopy -I srec -O binary ${S}/tools/dummy_create/cert_header_sa6.srec ${S}/tools/dummy_create/cert_header_sa6.bin
}

do_ipl_opt_deploy_append() {
	install -m 0644 ${S}/tools/dummy_create/bootparam_sa0.bin ${DEPLOYDIR}/bootparam_sa0-${EXTRA_ATFW_CONF}.bin
    install -m 0644 ${S}/tools/dummy_create/cert_header_sa6.bin ${DEPLOYDIR}/cert_header_sa6-${EXTRA_ATFW_CONF}.bin
}
