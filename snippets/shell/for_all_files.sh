#!/bin/bash
###############################################################
# list up files under the directory specified by the argument #
###############################################################

# remove a white space from a set of delimiters
# only a line feed and a tab character are used as delimiters
IFS_BUFF=$IFS
IFS=$'\n'$'\t'

# for loop to display each entry from the given command
for f in `ls -1 $1`
do
    echo "file: '$f'"
done

# recovery of the delimiter set
IFS=$IFS_BUFF

