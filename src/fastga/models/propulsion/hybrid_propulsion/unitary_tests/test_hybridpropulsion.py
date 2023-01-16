from openmdao.core.group import Group
import pytest
import numpy as np

from ..battery_parameters import BatteryParameters
from ..main import PropulsionHybrid
from ..motor import ComputeMotorMass
from ..switch import ComputeSwitchMass
from ..main import PropulsionHybrid
from ..fuelcell import FuelcellParameters

from tests.testing_utilities import run_system, get_indep_var_comp, list_inputs

XML_FILE = "input_hybrid_electric_ac.xml"


def test_hybrid_propulsion_group():
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(PropulsionHybrid(number_of_points=252)), __file__, XML_FILE)

    # Run problem and check obtained value(s) is/(are) correct
    problem = run_system(PropulsionHybrid(), ivc)
    total_weight = problem.get_val("data:propulsion:total_weight", units="kg")
    print("\ntotal weight is", total_weight)
