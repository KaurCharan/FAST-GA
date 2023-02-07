from openmdao.core.group import Group

import fastoad.api as oad
from fastoad.module_management.constants import ModelDomain
import openmdao.api as om
from fastga.models.performances.mission_vector.constants import SUBMODEL_ENERGY_CONSUMPTION

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


@oad.RegisterSubmodel(SUBMODEL_ENERGY_CONSUMPTION, "fastga.propulsion.hybrid_propulsion")
class PropulsionHybrid(om.Group):
    def initialize(self):
        self.options.declare("propulsion_id", default=None, types=str, allow_none=True)
        self.options.declare("number_of_points", default=252, types=int, allow_none=True)

    def setup(self):
        options_equilibrium = {"number_of_points": self.options["number_of_points"]}

        self.add_subsystem(
            "power_computation",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_COMPUTE_POWER, options_equilibrium),
            promotes=["*"],
        )

        self.add_subsystem(
            "motor_mass",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_MOTOR_MASS, options_equilibrium),
            promotes=["*"],
        )

        self.add_subsystem(
            "switch_mass",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_SWITCH_MASS, options_equilibrium),
            promotes=["*"]
        )

        self.add_subsystem(
            "cables_mass",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_CABLES_MASS),
            promotes=["*"]
        )

        self.add_subsystem(
            "converter_mass",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_CONVERTER_MASS, options_equilibrium),
            promotes=["*"]
        )

        self.add_subsystem(
            "fuelcell_parameter",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_FUELCELL_PARAMETERS, options_equilibrium),
            promotes=["*"]
        )

        self.add_subsystem(
            "battery_parameters",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_BATTERY_PARAMETERS, options_equilibrium),
            promotes=["*"]
        )

        self.add_subsystem(
            "summation_weights",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_SUMMATION_WEIGHTS),
            promotes=["*"]
        )
