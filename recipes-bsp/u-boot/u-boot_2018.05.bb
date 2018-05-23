require u-boot-common.inc
require u-boot.inc

DEPENDS += "bc-native dtc-native coreutils-native"

do_compile_append() {
	objcopy --srec-forceS3 --change-address 0x50000000 -I binary -O srec ${B}/u-boot.bin ${B}/u-boot-${MACHINE}.srec
}

do_deploy_append() {
	install -m 644 "${B}/u-boot-${MACHINE}.srec" "${DEPLOYDIR}/u-boot-${MACHINE}.srec"
}
