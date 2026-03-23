# LADYF

Pour Fusion 360, les API sont en codes python.
Pour installer les modules externes compatibles avec Fusion, suivre le liens comme guide:
https://dcyoung.github.io/post-f360-py-modules/

import os
import sys
import subprocess
subprocess.check_call([os.path.join(sys.path[0], "Python", "python.exe"), "-m", "pip", "install", "--upgrade", "numpy"])
import numpy as np
