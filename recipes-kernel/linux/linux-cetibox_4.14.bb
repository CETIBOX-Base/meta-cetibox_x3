DESCRIPTION = "Linux kernel for the CETiBOX X3"

require recipes-kernel/linux/linux-yocto.inc

# Let other packages from the meta-renesas layer reference our package. This
# makes the linux-renesas recipe obsolete.
PROVIDES += "linux-renesas"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}/:"
COMPATIBLE_MACHINE = "cetibox-h3"

# The internal CETiTEC git repository for the linux kernel. The probably most
# important parameter is 'protocol', which defines the protocol being used for
# cloning the remote repository.
CETIBOX_KERNEL_URL = "git://github.com/CETIBOX-Base/linux.git;protocol=https"

# You also may use a local git repository during development. This way, you
# don't need to push experimental changes to the kernel sources.
#
# Note: you must not use ssh if you want to use a local kernel repository.
#CETIBOX_KERNEL_URL = "git:///home/ubuntu/data/git/cetibox_x3/dev/code/components/linux/kernel"

# Use the update_submodule_recipes.sh script to update this revision
SRCREV = "460c31f8140cdb165b889f3c4f1125c169cea800"

# For development work, it can be useful to refer to the branch tip instead of a
# fixed commit. To enable this, uncomment the following lines and comment out the
# SRCREV above.
#SRCREV = "${AUTOREV}"
BRANCH = "v4.14/rcar-3.9-ctc"

# This variable contains, besides the git repository url, a few parameters
# which influence the git clone process.
#SRC_URI = "${CETIBOX_KERNEL_URL};nocheckout=1;rev=${SRCREV};nobranch=1"
SRC_URI = "${CETIBOX_KERNEL_URL};nocheckout=1;rev=${SRCREV};branch=${BRANCH}"

LINUX_VERSION ?= "4.14.201"
PV = "${LINUX_VERSION}+git${SRCPV}"
PR = "r1"


# This defines the name of the in-tree defconfig being used for building the
# kernel.
# Note: This defconfig _must_ be a real defconfig. In the original bsp from
#       Renesas an out-of-tree defconfig was used, which itself wasn't a real
#       defconfig but a complete .config file. This resulted in a lot of
#       confusion because another tool, 'configme', was used to combine several
#       kernel configuration fragments into one single configuration file.
#       We now provide a _real_ defconfig within the kernel source tree and
#       take care that the 'configme' does not interfere anymore in an unwanted
#       way. See also the notes on 'KCONFIG_MODE'.
KBUILD_DEFCONFIG = "cetibox_x3_defconfig"

# This is an undocumented configuration variable which influences the behaviour
# of the 'configme' script, which itself is part of the yocto toolchain. We
# need to set this variable to "--alldefconfig" to ensure that 'configme'
# behaves like a "make defconfig". Otherwise, all kernel options which are
# _not_ mentioned in the defconfig file are set to NO, which breaks a lot of
# things.
KCONFIG_MODE = "--alldefconfig"

# Gives the kernel an appropriate name extension.
LINUX_VERSION_EXTENSION = "-cetibox"

# Remove intermediate defconfigs, and any defconfig or .config the
# user has lying around in their working copy. The existence of any of
# these files causes the build process to use these files instead of
# the desired configuration from arch/arm64/configs/cetibox_x3_defconfig.
do_kernel_metadata_prepend () {
	rm -f ${WORKDIR}/defconfig
	rm -f ${S}/defconfig
	rm -f ${S}/.config
}

LINUX_FIRMWARE_VERSION = "20200918"

# Install USB3.0 firmware to rootfs
USB3_FIRMWARE_V2 = "https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/r8a779x_usb3_v2.dlmem?h=${LINUX_FIRMWARE_VERSION};name=r8a779x_usb3_v2;downloadfilename=r8a779x_usb3_v2.dlmem"
USB3_FIRMWARE_V3 = "https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/r8a779x_usb3_v3.dlmem?h=${LINUX_FIRMWARE_VERSION};name=r8a779x_usb3_v3;downloadfilename=r8a779x_usb3_v3.dlmem"

# Install TI WLAN firmware to rootfs
WL18XX_FW_4 = "https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/ti-connectivity/wl18xx-fw-4.bin?h=${LINUX_FIRMWARE_VERSION};name=wl18xx_fw_4;downloadfilename=wl18xx-fw-4.bin"

SRC_URI_append = " \
    ${USB3_FIRMWARE_V2} \
    ${USB3_FIRMWARE_V3} \
	${WL18XX_FW_4} \
	file://firmware.conf \
	file://wl18xx-conf.bin \
"

SRC_URI[r8a779x_usb3_v2.sha256sum] = "c401eb2dfe69a8817018d244cf84680dbca3e42ee068dc459a2bebe5088a40cb"
SRC_URI[r8a779x_usb3_v3.sha256sum] = "c49d294f321801a9a5c767c3a5b8e7be3fccec6f87808dc70a7d776c760eeea6"
SRC_URI[wl18xx_fw_4.sha256sum] = "95f04731257febabc358b95ff06dbb2025d047c6ad3ae0239c8afb92dedc7c5c"

do_download_firmware () {
    install -m 755 ${WORKDIR}/r8a779x_usb3_v*.dlmem ${STAGING_KERNEL_DIR}/firmware
}

addtask do_download_firmware after do_configure before do_compile

DEPENDS_append = " u-boot-mkimage-native"

do_compile_append() {
	usr/gen_init_cpio ${WORKDIR}/firmware.conf | gzip > firmware.cpio.gz
	mkimage -A arm -T ramdisk -C gzip -d firmware.cpio.gz firmware.img
}

do_install_append () {
	install -d ${D}${nonarch_base_libdir}/firmware
	install -m 755 ${WORKDIR}/r8a779x_usb3_v*.dlmem ${D}${nonarch_base_libdir}/firmware
	install -m 755 -d ${D}${nonarch_base_libdir}/firmware/ti-connectivity
	install -m 755 ${WORKDIR}/wl18xx-conf.bin ${D}${nonarch_base_libdir}/firmware/ti-connectivity
	install -m 755 ${WORKDIR}/wl18xx-fw-4.bin ${D}${nonarch_base_libdir}/firmware/ti-connectivity

	install -D -m 644 ${B}/firmware.img ${D}/boot/firmware.img
}

PACKAGES_append = " ${KERNEL_PACKAGE_NAME}-firmware-r8a779x-usb3 ${KERNEL_PACKAGE_NAME}-firmware-wl18xx"
FILES_${KERNEL_PACKAGE_NAME}-firmware-r8a779x-usb3 = "${nonarch_base_libdir}/firmware/r8a779x_usb3_v*.dlmem"
FILES_${KERNEL_PACKAGE_NAME}-firmware-wl18xx = "${nonarch_base_libdir}/firmware/ti-connectivity/wl*.bin"

FILES_${KERNEL_PACKAGE_NAME}-image_append = "boot/firmware.img"
