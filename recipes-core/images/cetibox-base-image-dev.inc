# Install additional debugging and development tools
IMAGE_FEATURES += "dev-pkgs tools-debug tools-testapps tools-profile debug-tweaks"

# In the following list, please put exactly one item per line, indent with
# exactly one tab, place exactly one space between the item and the back-slash
# escaping the end of line and KEEP THE LIST IN LEXICOGRAPHIC ORDER.  This way,
# automatic merges will function smoothly.

IMAGE_INSTALL_append = " \
	can-utils \
	devmem2 \
	ethtool \
	glibc-dbg \
	i2c-tools \
	less \
	netcat-openbsd \
	procps \
	python-argparse \
	python-ctypes \
	python-misc \
	python-smbus \
	python-subprocess \
	python-terminal \
	python3-can \
	python3-setuptools \
	tcpdump \
	tmux \
	udev-rules \
	usbutils \
	v4l-utils \
	valgrind \
	vim \
"
