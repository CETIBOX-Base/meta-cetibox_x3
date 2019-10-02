require u-boot-common.inc
require u-boot.inc

DEPENDS += "bc-native dtc-native coreutils-native"

do_compile_append() {
	for config in ${UBOOT_MACHINE}; do
		objcopy --srec-forceS3 --change-address 0x50000000 -I binary -O srec ${B}/${config}/u-boot.bin ${B}/u-boot-${config}.srec
	done
}

do_deploy_append() {
	for config in ${UBOOT_MACHINE}; do
		shortconfig=${config#r8a7795_}
		shortconfig=${shortconfig%_defconfig}
		shortconfig=$(echo "$shortconfig" | sed s/_/-/)
		install -m 644 "${B}/u-boot-${config}.srec" "${DEPLOYDIR}/u-boot-${shortconfig}.srec"
	done
}
