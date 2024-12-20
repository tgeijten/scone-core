# sconetools -- helper module to find sconepy
# (C) Copyright Thomas Geijtenbeek
# This file is part of SCONE. For more information, see http://scone.software.

import sys
import os
import platform
import pathlib
import importlib.util

# if sys.version_info < (3,9) or sys.version_info >= (3,10):
#     raise Exception("sconepy only supports Python 3.9 -- current version: " + platform.python_version() )

path_to_sconepy = ""

def try_find_sconepy(pathlist):
    global path_to_sconepy
    if path_to_sconepy:
        return; # already found
    for path in pathlist:
        if path:
            if sorted(pathlib.Path(path).glob("sconepy*.*")):
                path_to_sconepy = str(path)
                return;

# search for sconepy in os-specific paths
if importlib.util.find_spec("sconepy") is None:
    path_list = []

    if sys.platform.startswith("win"):
        if scone_install := os.getenv("SCONE_PATH"):
            path_list.append(scone_install + "/bin")
        path_list.extend([
            os.getenv("LOCALAPPDATA") + "/SCONE/bin",
            os.getenv("ProgramFiles") + "/SCONE/bin",
            ])
    elif sys.platform.startswith("linux"):
        if scone_install := os.getenv("SCONE_PATH"):
            path_list.append(scone_install + "/lib")
        path_list.extend([
            "/opt/scone-core/lib",
            pathlib.Path.home() / "scone-core/lib",
            "/opt/scone/lib",
            pathlib.Path.home() / "scone/lib"
            ])
    elif sys.platform.startswith("darwin"):
        if scone_install := os.getenv("SCONE_PATH"):
            path_list.append(scone_install + "/lib")
        path_list.extend([
            "/Applications/SCONE.app/Contents/MacOS/lib",
            pathlib.Path.home() / "SCONE.app/Contents/MacOS/lib"
            ])

    # find sconepy in path_list
    try_find_sconepy(path_list)

    # check if we succeeded
    if path_to_sconepy:
        print("Found SconePy at", path_to_sconepy)
        sys.path.append(path_to_sconepy)
    else:
        print("Could not find SconePy in", path_list)
        raise Exception("Could not find SconePy in " + path_list)

import sconepy