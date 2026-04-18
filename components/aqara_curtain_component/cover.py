# Compatibility shim for ESPHome versions that still load <component>/cover.py.
# Re-export the package-based implementation at <component>/cover/__init__.py.
from .cover import *  # noqa: F401,F403
