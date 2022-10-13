import sys
import platform
import pathlib

if sys.version_info < (3,9) or sys.version_info >= (3,10):
    raise Exception('SconePy only supports Python 3.9 -- current version: ' + platform.python_version() )

path_to_sconepy = ''

def try_find_sconepy(path):
    global path_to_sconepy
    if path_to_sconepy:
        return; # already found
    if sorted(pathlib.Path(path).glob('sconepy*.*')):
        path_to_sconepy = str(path)

if sys.platform.startswith('win'):
    try_find_sconepy('C:/Program Files/SCONE/bin')
    try_find_sconepy('D:/Build/scone-studio/vc2019-x64/bin/Release')
elif sys.platform.startswith('linux'):
    try_find_sconepy('/opt/scone-core/lib')
    try_find_sconepy(pathlib.Path.home() / 'scone-core/lib')

if path_to_sconepy:
    print('SconePy found at', path_to_sconepy)
    sys.path.append(path_to_sconepy)
else:
    raise Exception('Could not find SconePy')

import sconepy