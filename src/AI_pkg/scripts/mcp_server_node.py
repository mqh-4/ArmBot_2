#!/usr/bin/env python3
"""ROS executable; select the isolated AI interpreter in stock Noetic."""
import os
import sys

if sys.version_info < (3, 10):
    python = os.environ.get('AI_PYTHON', '/opt/ai-python/bin/python')
    if not os.path.isfile(python):
        raise SystemExit('AI runtime is missing. Run bash src/AI_pkg/scripts/install_ai_runtime.sh first.')
    os.execv(python, [python, __file__] + sys.argv[1:])

from ai_pkg.nodes import mcp_main


if __name__ == '__main__':
    mcp_main()
