import sys
import glob
import pathlib

path_to_sconepy = ''

def try_find_sconepy(path):
    global path_to_sconepy
    if path_to_sconepy:
        return; # already found
    if sorted(pathlib.Path(path).glob('sconepy*.*')):
        path_to_sconepy = str(path)

try:
    # check if sconepy is already in path
    import sconepy
except ModuleNotFoundError:
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