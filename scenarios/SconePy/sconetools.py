import sys
import platform
import pathlib
import importlib.util

# if sys.version_info < (3,9) or sys.version_info >= (3,10):
#     raise Exception('sconepy only supports Python 3.9 -- current version: ' + platform.python_version() )

path_to_sconepy = ''

def try_find_sconepy(path):
    global path_to_sconepy
    if path_to_sconepy:
        return; # already found
    if sorted(pathlib.Path(path).glob('sconepy*.*')):
        path_to_sconepy = str(path)

if importlib.util.find_spec('sconepy') is None:
    if sys.platform.startswith('win'):
        try_find_sconepy('C:/Program Files/SCONE/bin')
        try_find_sconepy('D:/Build/scone-studio/vc2019-x64/bin/Release')
    elif sys.platform.startswith('linux'):
        try_find_sconepy('/opt/scone-core/lib')
        try_find_sconepy(pathlib.Path.home() / 'scone-core/lib')
        try_find_sconepy('/opt/scone/lib')
        try_find_sconepy(pathlib.Path.home() / 'scone/lib')

    if path_to_sconepy:
        print('sconepy found at', path_to_sconepy)
        sys.path.append(path_to_sconepy)
    else:
        raise Exception('Could not find sconepy')

import sconepy