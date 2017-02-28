#!/bin/bash -e
# Copyright by California Institute of Technology, University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/cmcghan/vagrant-rss

#
# in the initialization process for vagrant, this bash script is run as user 'root' from /vagrant
#

# there is some cleaner stuff we can do here between scripts that we are enacting now
#
# basically, if we run something as "source my_script.sh" or ". ./myscript.sh",
# then it is run at the same "level" as the calling shell (or script) and changes are
# persistent, e.g.,
# -- "source" the subscript in a top-level script for the subscript to see all the
#    environment variables in that script
# -- modify an environment variable in the subscript for the top-level script that
#    "source"d it to see the change in the top-level script
#
# if you only want to share a few env vars from the top-level down, and make
# no changes to the env vars above:
# -- "export" a environment variable in the top-level script for the subscript to see
#    only that environment variable
# -- do not "source" the subscript for changes in the subscript not to affect the
#    top-level script
# see: http://stackoverflow.com/questions/9772036/pass-all-variables-from-one-shellscript-to-another

echo "Start of get_rv_su_wd_f.sh script!"
echo "sets ROSVERSION, SCRIPTUSER, WORKSPACEDIR, FORCE environment variable for a script 'source'ing it"

#
# parse input vars (set to appropriate vars or default vars)
#







echo "End of get_rv_su_wd_f.sh script!"
