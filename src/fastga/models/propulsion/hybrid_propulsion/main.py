from openmdao.core.group import Group

import fastoad.api as oad
from fastoad.module_management.constants import ModelDomain
import openmdao.api as om

from .constants import (
    SUBMODEL_MOTOR_MASS,
    SUBMODEL_BATTERY_PARAMETERS,
    SUBMODEL_SWITCH_MASS,
    SUBMODEL_COMPUTE_POWER,
    SUBMODEL_FUELCELL_PARAMETERS,
    SUBMODEL_SUMMATION_WEIGHTS,
    SUBMODEL_CABLES_MASS,
    SUBMODEL_CONVERTER_MASS
)


@oad.RegisterOpenMDAOSystem("fastga.propulsion.hybrid_propulsion", domain=ModelDomain.PROPULSION)
class PropulsionHybrid(om.Group):

    def setup(self):
        # options_equilibrium = {"number_of_points": 248}    # TO DO: comment during full oad process

        self.add_subsystem(
            "power_computation",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_COMPUTE_POWER),
            promotes=["*"],
        )

        self.add_subsystem(
            "motor_mass",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_MOTOR_MASS),
            promotes=["*"],
        )

        self.add_subsystem(
            "switch_mass",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_SWITCH_MASS),
            promotes=["*"]
        )

        self.add_subsystem(
            "cables_mass",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_CABLES_MASS),
            promotes=["*"]
        )

        self.add_subsystem(
            "converter_mass",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_CONVERTER_MASS),
            promotes=["*"]
        )

        self.add_subsystem(
            "fuelcell_parameter",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_FUELCELL_PARAMETERS),
            promotes=["*"]
        )  # TO DO: remove "options" during full oad process

        self.add_subsystem(
            "battery_parameters",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_BATTERY_PARAMETERS),
            promotes=["*"]
        )  # TO DO: remove "options" during full oad process

        self.add_subsystem(
            "summation_weights",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_SUMMATION_WEIGHTS),
            promotes=["*"]
        )
