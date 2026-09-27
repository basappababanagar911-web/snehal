"""
integrate.py
============
SteadyPath Master Integration Launcher
Entry point forwarding execution to scripts/integrate.py

Usage:
    python integrate.py --controller mpc --scenario normal
    python integrate.py --controller stanley --scenario curved
    python integrate.py --controller mpc --scenario replan
    python integrate.py --controller mpc --scenario moving
    python integrate.py --controller mpc --scenario all
"""

import os
import sys

# Ensure root and scripts directory are accessible
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(PROJECT_ROOT, "scripts")

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from scripts.integrate import main

if __name__ == "__main__":
    main()
