"""
Test module for mass breakdown functions.
"""
#  This file is part of FAST-OAD_CS23 : A framework for rapid Overall Aircraft Design
#  Copyright (C) 2022  ONERA & ISAE-SUPAERO
#  FAST is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

import pytest

from ..b_propulsion import (
    ComputeHydrogenStorageWeight,
    ComputeFuelCellWeight,
)
from ..b_propulsion.sum import PropulsionWeight

from ..a_airframe.wing_components import ComputePoddedHydrogenStorage

from tests.testing_utilities import run_system, get_indep_var_comp, list_inputs

from .dummy_engines import ENGINE_WRAPPER_BE76 as ENGINE_WRAPPER

XML_FILE = "hybrid_electric_ac.xml"

def test_hydrogen_storage_weight():
    """Tests hydrogen storage weight computation from sample XML data."""
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(ComputeHydrogenStorageWeight()), __file__, XML_FILE)

def test_fuell_cell_weight():
    """Tests hydrogen storage weight computation from sample XML data."""
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(ComputeFuelCellWeight()), __file__, XML_FILE)

def test_compute_propulsion_weight():
    """Tests propulsion weight computation from sample XML data."""
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(
        list_inputs(PropulsionWeight(propulsion_id=ENGINE_WRAPPER)), __file__, XML_FILE
    )

def test_compute_podded_hydrogen_storage():
    """Tests propulsion weight computation from sample XML data."""
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(
        list_inputs(ComputePoddedHydrogenStorage()), __file__, XML_FILE
    )