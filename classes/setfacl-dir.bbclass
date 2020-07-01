# SETFACL_DIR - directory to set ACLs on, will have owner/group root,
#   if it does not already exist it will be created
# SETFACL_ACL - ACL entries to set on SETFACL_DIR, the entries will be installed
#   as default ACL on SETFACL_DIR and recursively applied on the
#   contents of SETFACL_DIR.

SETFACL_ACL_ACTUAL = ""
python __anonymous () {
    acl_specs = d.getVar('SETFACL_ACL').split()
    spec = ','.join(acl_specs)
    d.setVar('SETFACL_ACL_ACTUAL', spec)
}

do_install_append() {
   install -d -m 0770 -o root -g root ${D}${SETFACL_DIR}
}

PACKAGE_WRITE_DEPS += "acl-native"
pkg_postinst_append_${PN}() {

exported_dir=${SETFACL_DIR}
acl=${SETFACL_ACL_ACTUAL}
echo SETFACL-DIR "$acl" $D"$exported_dir"

# determine if we are in a sysroot and
# ensure pseudo uses the sysroots password database
if [ -n "$D" ]; then
    export PSEUDO_PASSWD="$D"
fi

# setup default ACL
setfacl -dm "$acl" $D"$exported_dir"
# setup ACL recursively
setfacl -Rm "$acl" $D"$exported_dir"

echo SETFACL-DIR complete

}
