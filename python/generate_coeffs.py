#!/usr/bin/env python3

import os
import subprocess
import sys

subprocess.run([sys.executable, "flywheel.py", "--noninteractive"])
os.rename("ShooterCoeffs.h", "../src/include/Subsystems/ShooterCoeffs.h")
os.rename("ShooterCoeffs.cpp", "../src/cpp/Subsystems/ShooterCoeffs.cpp")
