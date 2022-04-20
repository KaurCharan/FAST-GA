"""
Test module for basicTP_engine_constructor.py
"""

#  This file is part of FAST : A framework for rapid Overall Aircraft Design
#  Copyright (C) 2020  ONERA & ISAE-SUPAERO
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

import numpy as np

from ..basicTP_engine_constructor import ComputeTurbopropMap

from tests.testing_utilities import run_system, get_indep_var_comp, list_inputs

from .data.dummy_maps import (
    MACH_ARRAY,
    THRUST_ARRAY_CL,
    THRUST_ARRAY_IL,
    THRUST_ARRAY_SL,
    THRUST_MAX_ARRAY_CL,
    THRUST_MAX_ARRAY_IL,
    THRUST_MAX_ARRAY_SL,
    SFC_SL,
    SFC_CL,
    SFC_IL,
)

XML_FILE = "input_be_king_air.xml"


def test_table_construction():
    """Tests the construction of the table"""

    # Research independent input value in .xml file and add values calculated from other modules
    ivc = get_indep_var_comp(list_inputs(ComputeTurbopropMap()), __file__, XML_FILE)

    # Run problem and check obtained value(s) is/(are) correct
    problem = run_system(ComputeTurbopropMap(), ivc)

    assert (
        np.sum(np.abs(MACH_ARRAY - problem.get_val("data:propulsion:turboprop:sea_level:mach")))
        < 1e-4
    )
    assert (
        np.sum(
            np.abs(
                THRUST_ARRAY_SL
                - problem.get_val("data:propulsion:turboprop:sea_level:thrust", units="N")
            )
        )
        < 1e-2
    )
    assert (
        np.sum(
            np.abs(
                THRUST_MAX_ARRAY_SL
                - problem.get_val("data:propulsion:turboprop:sea_level:thrust_limit", units="N")
            )
        )
        < 1e-2
    )
    assert (
        np.sum(
            np.abs(
                SFC_SL - problem.get_val("data:propulsion:turboprop:sea_level:sfc", units="kg/s/N")
            )
        )
        < 1e-2
    )

    assert (
        np.sum(np.abs(MACH_ARRAY - problem.get_val("data:propulsion:turboprop:cruise_level:mach")))
        < 1e-4
    )
    assert (
        np.sum(
            np.abs(
                THRUST_ARRAY_CL
                - problem.get_val("data:propulsion:turboprop:cruise_level:thrust", units="N")
            )
        )
        < 1e-2
    )
    assert (
        np.sum(
            np.abs(
                THRUST_MAX_ARRAY_CL
                - problem.get_val("data:propulsion:turboprop:cruise_level:thrust_limit", units="N")
            )
        )
        < 1e-2
    )
    assert (
        np.sum(
            np.abs(
                SFC_CL
                - problem.get_val("data:propulsion:turboprop:cruise_level:sfc", units="kg/s/N")
            )
        )
        < 1e-2
    )

    assert (
        np.sum(
            np.abs(
                MACH_ARRAY - problem.get_val("data:propulsion:turboprop:intermediate_level:mach")
            )
        )
        < 1e-4
    )
    assert (
        np.sum(
            np.abs(
                THRUST_ARRAY_IL
                - problem.get_val("data:propulsion:turboprop:intermediate_level:thrust", units="N")
            )
        )
        < 1e-2
    )
    assert (
        np.sum(
            np.abs(
                THRUST_MAX_ARRAY_IL
                - problem.get_val(
                    "data:propulsion:turboprop:intermediate_level:thrust_limit", units="N"
                )
            )
        )
        < 1e-2
    )
    assert (
        np.sum(
            np.abs(
                SFC_IL
                - problem.get_val(
                    "data:propulsion:turboprop:intermediate_level:sfc", units="kg/s/N"
                )
            )
        )
        < 1e-2
    )
