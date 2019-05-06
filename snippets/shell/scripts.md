# Cheetsheets of some bash scripts

# list up files under the directory specified by the argument
```
#!/bin/bash
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
```

# replace phrases in each file
```
#!/bin/bash
# sh <this script> <current path> <phrase to search> <phrase to add>

for f in `ls -1 $1`
do
    sed -i 's#'$2'#'$3'#g' $f
done
```