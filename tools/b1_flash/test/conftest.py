import sys
from pathlib import Path

# The package lives next to this directory, not on any install path.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
