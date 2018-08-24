#!/usr/bin/env python3

import os
import subprocess
import sys

subprocess.run([sys.executable, "elevator.py", "--noninteractive"])
os.rename("ElevatorCoeffs.h", "../src/include/Subsystems/ElevatorCoeffs.h")
os.rename("ElevatorCoeffs.cpp", "../src/cpp/Subsystems/ElevatorCoeffs.cpp")
