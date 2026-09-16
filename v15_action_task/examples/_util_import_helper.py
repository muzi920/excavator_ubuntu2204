import os
import sys


def ensure_v15_importable() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    examples_dir = here
    v15_parent = os.path.dirname(os.path.dirname(examples_dir))
    if v15_parent not in sys.path:
        sys.path.insert(0, v15_parent)
