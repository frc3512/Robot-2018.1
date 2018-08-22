#!/usr/bin/env python3

import os
import subprocess
import sys

subprocess.run([sys.executable, "drivetrain.py", "--noninteractive"])
os.rename("DrivetrainCoeffs.hpp", "../src/include/Control/DrivetrainCoeffs.hpp")
os.rename("DrivetrainCoeffs.cpp", "../src/cpp/Control/DrivetrainCoeffs.cpp")
