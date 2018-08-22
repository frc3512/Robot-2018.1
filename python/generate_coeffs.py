#!/usr/bin/env python3

import os
import subprocess
import sys

subprocess.run([sys.executable, "drivetrain.py", "--noninteractive"])
os.rename("DrivetrainCoeffs.h", "../src/include/Subsystems/DrivetrainCoeffs.h")
os.rename("DrivetrainCoeffs.cpp", "../src/cpp/Subsystems/DrivetrainCoeffs.cpp")
