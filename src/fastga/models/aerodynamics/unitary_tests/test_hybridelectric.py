from platform import system

import numpy as np
import pytest

from .dummy_engines import ENGINE_WRAPPER_SR22 as ENGINE_WRAPPER
from .test_functions import (
    cd_inlets,
)

XML_FILE = "hybrid_electric.xml"
SKIP_STEPS = True  # avoid some tests to accelerate validation process (intermediary VLM/OpenVSP)


def test_cd_inlets():
    """Tests maximum/minimum cl component with default result cl=f(y) curve."""
    cd_inlets(XML_FILE)

