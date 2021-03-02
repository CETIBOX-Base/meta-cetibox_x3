# Install docker and its dependencies.
#
# Note:
# Make sure that DISTRO_FEATURES contains " virtualization". This is has to be
# done in the distro configuration.

IMAGE_INSTALL_append = " \
	docker \
"
