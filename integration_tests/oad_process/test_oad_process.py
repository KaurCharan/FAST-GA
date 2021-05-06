"""
Test module for Overall Aircraft Design process
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

import os
import os.path as pth
from shutil import rmtree

import openmdao.api as om
import pytest
from numpy.testing import assert_allclose

from fastoad.io.configuration.configuration import FASTOADProblemConfigurator
# from fastoad import api

DATA_FOLDER_PATH = pth.join(pth.dirname(__file__), "data")
RESULTS_FOLDER_PATH = pth.join(pth.dirname(__file__), "results")
PATH = pth.dirname(__file__).split(os.path.sep)
NOTEBOOKS_PATH = PATH[0] + os.path.sep
for folder in PATH[1:len(PATH) - 3]:
    NOTEBOOKS_PATH = pth.join(NOTEBOOKS_PATH, folder)
NOTEBOOKS_PATH = pth.join(NOTEBOOKS_PATH, "notebooks")

AIRCRAFT_ID = "sr22"  # "be76"
MDA_WING_POSITION = True


@pytest.fixture(scope="module")
def cleanup():
    rmtree(RESULTS_FOLDER_PATH, ignore_errors=True)
    rmtree("D:/tmp", ignore_errors=True)


def test_oad_process(cleanup):
    """
    Test the overall aircraft design process without wing positioning.
    """

    # Define used files depending on options
    if MDA_WING_POSITION:
        xml_file_name = 'input_' + AIRCRAFT_ID + '_wing_pos.xml'
        process_file_name = 'oad_process_' + AIRCRAFT_ID + '_wing_pos.yml'
    else:
        xml_file_name = 'input_' + AIRCRAFT_ID + '.xml'
        process_file_name = 'oad_process_' + AIRCRAFT_ID + '.yml'

    configurator = FASTOADProblemConfigurator(pth.join(DATA_FOLDER_PATH, process_file_name))

    # Create inputs
    ref_inputs = pth.join(DATA_FOLDER_PATH, xml_file_name)
    # api.list_modules(pth.join(DATA_FOLDER_PATH, process_file_name), force_text_output=True)
    configurator.write_needed_inputs(ref_inputs)

    # Create problems with inputs
    problem = configurator.get_problem(read_inputs=True)
    problem.setup()
    problem.run_model()
    problem.write_outputs()

    if not pth.exists(RESULTS_FOLDER_PATH):
        os.mkdir(RESULTS_FOLDER_PATH)
    om.view_connections(
        problem, outfile=pth.join(RESULTS_FOLDER_PATH, "connections.html"), show_browser=False
    )
    om.n2(problem, outfile=pth.join(RESULTS_FOLDER_PATH, "n2.html"), show_browser=False)

    # Check that weight-performances loop correctly converged
    _check_weight_performance_loop(problem)

    if MDA_WING_POSITION:
        if AIRCRAFT_ID == "sr22":
            # noinspection PyTypeChecker
            assert_allclose(problem.get_val("data:mission:sizing:fuel", units="kg"), 250, atol=1)
            assert_allclose(problem["data:handling_qualities:stick_fixed_static_margin"], 0.10, atol=1e-2)
            # noinspection PyTypeChecker
            assert_allclose(problem.get_val("data:weight:aircraft:MTOW", units="kg"), 1648, atol=1)
            # noinspection PyTypeChecker
            assert_allclose(problem.get_val("data:weight:aircraft:OWE", units="kg"), 1043, atol=1)
        else:
            # noinspection PyTypeChecker
            assert_allclose(problem.get_val("data:mission:sizing:fuel", units="kg"), 218., atol=1)
            assert_allclose(problem["data:handling_qualities:stick_fixed_static_margin"], 0.15, atol=1e-2)
            # noinspection PyTypeChecker
            assert_allclose(problem.get_val("data:weight:aircraft:MTOW", units="kg"), 1721., atol=1)
            # noinspection PyTypeChecker
            assert_allclose(problem.get_val("data:weight:aircraft:OWE", units="kg"), 1113., atol=1)


def _check_weight_performance_loop(problem):
    assert_allclose(
        problem["data:weight:aircraft:OWE"],
        problem["data:weight:airframe:mass"]
        + problem["data:weight:propulsion:mass"]
        + problem["data:weight:systems:mass"]
        + problem["data:weight:furniture:mass"],
        rtol=5e-2,
    )
    assert_allclose(
        problem["data:weight:aircraft:MZFW"],
        problem["data:weight:aircraft:OWE"] + problem["data:weight:aircraft:max_payload"],
        rtol=5e-2,
    )
    assert_allclose(
        problem["data:weight:aircraft:MTOW"],
        problem["data:weight:aircraft:OWE"]
        + problem["data:weight:aircraft:payload"]
        + problem["data:mission:sizing:fuel"],
        rtol=5e-2,
    )
