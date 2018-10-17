require recipes-graphics/images/core-image-renesas-base.inc
require recipes-core/images/core-image-minimal.bb

IMAGE_FEATURES += "ssh-server-dropbear"

IMAGE_INSTALL_append = " \
	iproute2 \
	iproute2-tc \
	iproute2-ss \
	kernel-modules \
	most-driver \
	coreutils \
	openssh-sftp-server \
	u-boot-fw-utils \
	sja1105-tool \
	networkmanager \
	phytool \
	set-led \
	rsync \
	tcpdump \
	kernel-firmware-r8a779x-usb3 \
"

# Uncomment the following lines to install the UGW evaluation version to your
# image. Note that you need to copy the binary tarball 'UgwEval.tar' to the
# 'files' directory of the ugw-eval recipe.
# To get a copy of the UGW evaluation binary tarball, please contact us:
#   info@cetitec.com
#
#IMAGE_INSTALL_append = " \
#    ugw-eval \
#"

IMAGE_INSTALL_remove = " \
	libx11-locale \
"

# Don't install dev packages for openssh (conflicts with dropbear)
PACKAGE_EXCLUDE_COMPLEMENTARY = "openssh.*"
