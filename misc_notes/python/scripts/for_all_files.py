###########################################################
# displays files under the directory given by an argumrnt #
###########################################################

import os
import sys

if len(sys.argv) == 2:
	for entry in os.listdir(sys.argv[1]):
		print('file: ' + entry)

else:
	print('give a directory path')

print('end')


