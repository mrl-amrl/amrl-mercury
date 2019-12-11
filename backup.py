import os
import sys
import time
import pathspec
from zipfile import ZipFile

if not os.path.exists("backup"):
    os.mkdir("backup")

lines = []
with open('.gitignore', 'r') as file_handler:
    lines = file_handler.readlines()

lines.append('.git')
lines.remove('mercury-gui\n')

spec = pathspec.PathSpec.from_lines('gitwildmatch', lines)

zip_file = ZipFile(os.path.join(sys.argv[1], 'backup/{}.zip'.format(int(time.time()))), 'w')

for dirpath, dnames, fnames in os.walk("./"):
    for f in fnames:
        name = os.path.join(dirpath, f)
        if spec.match_file(name):
            continue

        zip_file.write(name)

zip_file.close()