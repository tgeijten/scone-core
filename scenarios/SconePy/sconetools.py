# sconetools -- helper module to find sconepy
# (C) Copyright Thomas Geijtenbeek
# This file is part of SCONE. For more information, see http://scone.software.

import sys
import platform
import pathlib
import importlib.util

# if sys.version_info < (3,9) or sys.version_info >= (3,10):
#     raise Exception('sconepy only supports Python 3.9 -- current version: ' + platform.python_version() )

path_to_sconepy = ''

def try_find_sconepy(pathlist):
    global path_to_sconepy
    if path_to_sconepy:
        return; # already found
    for path in pathlist:
        if sorted(pathlib.Path(path).glob('sconepy*.*')):
            path_to_sconepy = str(path)
            return;

if importlib.util.find_spec('sconepy') is None:
    if sys.platform.startswith('win'):
        try_find_sconepy([
            'C:/Program Files/SCONE/bin',
            'D:/Build/scone-studio/vc2019-x64/bin/Release'
            ])
    elif sys.platform.startswith('linux'):
        try_find_sconepy([
            '/opt/scone-core/lib',
            pathlib.Path.home() / 'scone-core/lib',
            '/opt/scone/lib',
            pathlib.Path.home() / 'scone/lib'
            ])

    if path_to_sconepy:
        print('sconepy found at', path_to_sconepy)
        sys.path.append(path_to_sconepy)
    else:
        raise Exception('Could not find sconepy')

import sconepy