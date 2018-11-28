# Install additional debugging and development tools
IMAGE_FEATURES += "dev-pkgs tools-debug tools-testapps tools-profile debug-tweaks"

IMAGE_INSTALL_append = " \
	vim \
	tmux \
	less \
	tcpdump \
	i2c-tools \
	can-utils \
	ethtool \
	procps \
	usbutils \
	devmem2 \
	python-smbus \
	python-importlib \
	python-misc \
	python-argparse \
	python-ctypes \
	python-subprocess \
	python-terminal \
	python3-can \
	python3-setuptools \
	v4l-utils \
	glibc-dbg \
	valgrind \
	udev-rules \
"
