#!/usr/bin/env python3

import os
import subprocess
import sys

subprocess.run([sys.executable, "drivetrain.py", "--noninteractive"])
os.rename("DrivetrainCoeffs.hpp",
          "../src/main/include/control/DrivetrainCoeffs.hpp")
os.rename("DrivetrainCoeffs.cpp",
          "../src/main/cpp/control/DrivetrainCoeffs.cpp")
