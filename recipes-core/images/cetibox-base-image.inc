require recipes-graphics/images/core-image-renesas-base.inc
require recipes-core/images/core-image-minimal.bb

COMPATIBLE_MACHINE = "cetibox-h3"

IMAGE_FEATURES += "ssh-server-dropbear"

# In the following list, please put exactly one item per line, indent with
# exactly one tab, place exactly one space between the item and the back-slash
# escaping the end of line and KEEP THE LIST IN LEXICOGRAPHIC ORDER.  This way,
# automatic merges will function smoothly.

IMAGE_INSTALL_append = " \
	coreutils \
	iproute2 \
	iproute2-ss \
	iproute2-tc \
	iw \
	kernel-firmware-r8a779x-usb3 \
	kernel-firmware-wl18xx \
	kernel-modules \
	most-driver \
	networkmanager \
	openssh-sftp-server \
	phytool \
	rsync \
	set-led \
	sja1105-tool \
	tcpdump \
	u-boot-fw-utils \
	udev-rules \
"

IMAGE_INSTALL_remove = " \
	libx11-locale \
"

# Don't install dev packages for openssh (conflicts with dropbear)
PACKAGE_EXCLUDE_COMPLEMENTARY = "openssh.*"
