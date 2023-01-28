"""Test module for geometry functions of cg components."""
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

from tests.testing_utilities import run_system, get_indep_var_comp, list_inputs

from ..cg_components.b_propulsion import (
    FuelPropulsionCG,
    ComputeHydrogenStorageCG,
    ComputeFuelCellCG,
)

from .dummy_engines import ENGINE_WRAPPER_BE76 as ENGINE_WRAPPER

XML_FILE = "hybrid_electric_ac.xml"

def test_compute_cg_fuel_propulsion():
    """Tests whole fuel propulsion center of gravity."""
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(FuelPropulsionCG()), __file__, XML_FILE)

def test_compute_cg_hydrogen_storage():
    """Tests whole hydrogen storage center of gravity."""
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(ComputeHydrogenStorageCG()), __file__, XML_FILE)

def test_compute_cg_fuel_cell():
    """Tests whole fuel cell center of gravity."""
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(ComputeFuelCellCG()), __file__, XML_FILE)