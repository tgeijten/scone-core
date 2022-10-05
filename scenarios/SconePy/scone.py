import sys

if sys.platform.startswith('win'):
    sys.path.append('C:/Program Files/SCONE/bin')
    sys.path.append('D:/Build/scone-studio/vc2019-x64/bin/Release')
elif sys.platform.startswith('linux'):
    sys.path.append('/opt/scone-core/lib')
else:
    raise Exception('Unsupported platform')

import sconepy