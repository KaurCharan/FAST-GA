import logging
import fastoad.api as oad
import numpy as np
import openmdao.api as om

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_CONVERTER_MASS
_LOGGER = logging.getLogger(__name__)


@oad.RegisterSubmodel(SUBMODEL_CONVERTER_MASS,
                      "fastga.submodel.propulsion.hybrid_propulsion.converter.legacy")
class ComputeConverterMass(om.ExplicitComponent):
    def initialize(self):
        self.options.declare(
            "number_of_points", default=252, desc="number of equilibrium to be treated"
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_input("mechanical_power", units="W", shape=number_of_points)
        self.add_input("data:mission:sizing:takeoff:power", units="W")
        self.add_input("motor_efficiency", val=0.93)
        self.add_input("battery_efficiency", val=0.98)
        self.add_input("switch_efficiency", val=0.97)
        self.add_input("gearbox_efficiency", val=0.98)
        self.add_input("controller_efficiency", val=0.97)
        self.add_input("bus_efficiency", val=0.97)
        self.add_input("converter_efficiency", val=0.97)
        self.add_input("cables_efficiency", val=0.99)
        self.add_input("data:propulsion:converter:power_to_mass_ratio")  #### check input

        self.add_output(
            "converter_weight",
            val=0.0,
            units="kg",
            desc="motor mass of all motors",
        )
        self.add_output("data:geometry:propulsion:converter:weight", units="kg", desc="motor mass of all motors")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        _LOGGER.debug("Calculating converter parameters")
        total_efficiency = inputs["motor_efficiency"] * inputs["gearbox_efficiency"] * inputs["controller_efficiency"] \
                           * inputs["switch_efficiency"] * inputs["bus_efficiency"] * inputs["converter_efficiency"]

        # Power supplied to DC-DC converter [W]
        mechanical_power_total = np.append(np.array(inputs["data:mission:sizing:takeoff:power"]),
                                           inputs["mechanical_power"])
        max_power = max(mechanical_power_total) / 1000  # in [kW]
        electrical_power = max_power / total_efficiency
        if any(ele > 1e6 for ele in electrical_power) == 1:
            mass_inv_conv = 600
        else:
            powerToMassRatio = inputs[
                "data:propulsion:converter:power_to_mass_ratio"]  # Power ro mass ratio of converter [W/kg]

            mass_inv_conv = electrical_power / powerToMassRatio

        outputs["converter_weight"] = mass_inv_conv
        outputs["data:geometry:propulsion:converter:weight"] = mass_inv_conv
