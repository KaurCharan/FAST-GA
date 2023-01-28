"""
Test module for geometry functions of cg components.
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

import openmdao.api as om
import pytest

from tests.testing_utilities import run_system, get_indep_var_comp, list_inputs

from .dummy_engines import ENGINE_WRAPPER_BE76 as ENGINE_WRAPPER

XML_FILE = "hybrid_electric_ac.xml"

from ..geom_components.fuselage.components import (
    ComputeFuselageGeometryCabinSizingH2
)

from ..geom_components.fuselage.compute_fuselage import ComputeFuselageH2

from ..geometry import GeometryHydrogenFuselage

def test_compute_fuselage_cabin_sizing_h2():
    ivc = get_indep_var_comp(
        list_inputs(ComputeFuselageGeometryCabinSizingH2(propulsion_id=ENGINE_WRAPPER)),
        __file__,
        XML_FILE,
    )

def test_compute_fuselage_h2():
    ivc = get_indep_var_comp(
        list_inputs(ComputeFuselageH2(propulsion_id=ENGINE_WRAPPER)),
        __file__,
        XML_FILE,
    )

def test_complete_geometry_hydrogen_fuselage():
    """Run computation of all models for fixed distance hypothesis"""

    # Research independent input value in .xml file and add values calculated from other modules
    # noinspection PyTypeChecker
    ivc = get_indep_var_comp(
        list_inputs(GeometryHydrogenFuselage(propulsion_id=ENGINE_WRAPPER)), __file__, XML_FILE
    )