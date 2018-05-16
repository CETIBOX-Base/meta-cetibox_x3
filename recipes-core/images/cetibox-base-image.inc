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

IMAGE_INSTALL_remove = " \
	libx11-locale \
"

# Don't install dev packages for openssh (conflicts with dropbear)
PACKAGE_EXCLUDE_COMPLEMENTARY = "openssh.*"
