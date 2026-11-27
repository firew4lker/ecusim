"""Entry point of the packaged (PyInstaller) binary; the logic lives in ecusim.cli."""

import sys

from ecusim.cli import main

if __name__ == "__main__":
    sys.exit(main())
