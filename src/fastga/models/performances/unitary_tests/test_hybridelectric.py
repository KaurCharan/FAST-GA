from openmdao.core.group import Group
import pytest
import numpy as np

from fastga.models.performances.mission.takeoff import (
    TakeOffPhase,
    _v2,
    _vr_from_v2,
    _v_lift_off_from_v2,
    _simulate_takeoff,
)
from fastga.models.performances.mission.mission_components import (
    ComputeTaxi,
    ComputeClimb,
    ComputeClimbSpeed,
    ComputeCruise,
    ComputeDescent,
    ComputeDescentSpeed,
    ComputeReserve,
)
from fastga.models.performances.mission.mission import Mission
from fastga.models.performances.mission.mission_builder_prep import PrepareMissionBuilder
from fastga.models.performances.mission_vector.mission_vector import MissionVector
from ..payload_range.payload_range import ComputePayloadRange

from tests.testing_utilities import run_system, get_indep_var_comp, list_inputs

from fastga.models.weight.cg.cg_variation import InFlightCGVariation

from .dummy_engines import ENGINE_WRAPPER_BE76 as ENGINE_WRAPPER


XML_FILE = "hybrid_electric.xml"
SKIP_STEPS = True


def test_mission_vector():

    # Research independent input value in .xml file
    ivc = get_indep_var_comp(
        list_inputs(MissionVector(propulsion_id=ENGINE_WRAPPER)),
        __file__,
        XML_FILE,
    )

    problem = run_system(
        MissionVector(propulsion_id=ENGINE_WRAPPER),
        ivc,
    )
    delta_Cl = problem.get_val("delta_Cl")
    delta_Cd = problem.get_val("delta_Cd")
    print(delta_Cl)
    print(delta_Cd)

