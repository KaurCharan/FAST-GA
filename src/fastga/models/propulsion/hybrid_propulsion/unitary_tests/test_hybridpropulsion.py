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


def test_compute_powerplant_mass():
    ivc = get_indep_var_comp(list_inputs(ComputeMotorMass()),__file__, XML_FILE)
    problem = run_system(PropulsionHybrid(), ivc)
    total_weight = problem.get_val("data:propulsion:total_weight", units="kg")

    print("\ntotal powerplant weight is", total_weight)


def test_compute_battery_parameters():
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(__file__, XML_FILE)

    # Run problem and check obtained value(s) is/(are) correct
    problem = run_system(PropulsionHybrid(), ivc)
    total_weight = problem.get_val("data:propulsion:total_weight", units="kg")

    print("\ntotal powerplant weight is", total_weight)


def test_compute_motor_weight():
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(ComputeMotorMass()), __file__, XML_FILE)

    # Run problem and check obtained value(s) is/(are) correct
    problem = run_system(ComputeMotorMass(), ivc)
    motor_weight = problem.get_val("data:geometry:propulsion:motor:weight", units="kg")
    print("\nmotor weight is", motor_weight)


def test_compute_switch_weight():
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(ComputeSwitchMass()), __file__, XML_FILE)

    # Run problem and check obtained value(s) is/(are) correct
    problem = run_system(ComputeSwitchMass(), ivc)
    switch_weight = problem.get_val("data:geometry:propulsion:switch:weight", units="kg")
    print("\nswitch weight is", switch_weight)


def test_compute_fuelcell_parameters():
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(FuelcellParameters(V_max, TO_Pelec, Climb_Pelec, Cruise_Pelec, TO_time,
                                                            Climb_time, Cruise_time, P_oper,
                                                            P_nom, single_stack_mass, cells_in_1stack, lambdaO2,
                                                            H2_density_300bar, H2_density_700bar,
                                                            H2_density_liq, H2_storageEff_300bar, H2_storageEff_700bar,
                                                            H2_storageEff_liq, H2_volumeEff_300bar,
                                                            H2_volumeEff_700bar, H2_volumeEff_liq, margin, V_thermo,
                                                            Top, Rgas, n, F, R, alpha, Io, I_leak,
                                                            I_limit, c, mu_H2)), __file__, XML_FILE)

    # Run problem and check obtained value(s) is/(are) correct
    problem = run_system(FuelcellParameters(), ivc)
    switch_weight = problem.get_val("data:geometry:propulsion:switch:weight", units="kg")
    print("\nswitch weight is", switch_weight)


def test_hybrid_propulsion_group():
    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(PropulsionHybrid()), __file__, XML_FILE)

    # Run problem and check obtained value(s) is/(are) correct
    problem = run_system(PropulsionHybrid(), ivc)
    total_weight = problem.get_val("data:geometry:propulsion:total_weight", units="kg")
    print("\ntotal weight is", total_weight)
