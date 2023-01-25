import fastoad.api as oad
import numpy as np
import openmdao.api as om

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_CONVERTER_MASS


@oad.RegisterSubmodel(SUBMODEL_CONVERTER_MASS,
                      "fastga.submodel.propulsion.hybrid_propulsion.converter.legacy")
class ComputeConverterMass(om.ExplicitComponent):
    def initialize(self):
        self.options.declare(
            "number_of_points", default=252, desc="number of equilibrium to be treated"
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_input("mechanical_power", shape=number_of_points)
        self.add_input("data:mission:sizing:takeoff:power")
        self.add_input("motor_efficiency", val=0.85)
        self.add_input("switch_efficiency", val=0.95)
        self.add_input("gearbox_efficiency", val=0.98)
        self.add_input("controller_efficiency", val=0.95)
        self.add_input("bus_efficiency", val=0.95)
        self.add_input("converter_efficiency", val=0.90)
        self.add_input("data:geometry:propulsion:fuelcell:stacks")  #### check input
        self.add_input("data:geometry:propulsion:battery:stacks")  ### check input
        self.add_input("data:propulsion:converter:power_to_mass_ratio")  #### check input

        self.add_output(
            "converter_weight",
            val=0.0,
            units="kg",
            desc="motor mass of all motors",
        )
        self.add_output("data:geometry:propulsion:converter:weight", units="kg", desc="motor mass of all motors")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        total_efficiency = inputs["motor_efficiency"] * inputs["gearbox_efficiency"] * inputs["controller_efficiency"] \
                           * inputs["switch_efficiency"] * inputs["bus_efficiency"] * inputs["converter_efficiency"]

        # Power supplied to DC-DC converter [W]
        mechanical_power_total = np.append(np.array(inputs["data:mission:sizing:takeoff:power"]),
                                           inputs["mechanical_power"])
        max_power = max(mechanical_power_total)
        electrical_power = max_power / total_efficiency

        NfuelCell = inputs["data:geometry:propulsion:fuelcell:stacks"]  # Number of fuel cell stacks
        Nbatt = inputs["data:geometry:propulsion:battery:stacks"]  # Number of battery stacks
        powerToMassRatio = inputs[
            "data:propulsion:converter:power_to_mass_ratio"]  # Power ro mass ratio of converter [W/kg]
        mass_inv_conv = (electrical_power * NfuelCell + electrical_power * Nbatt) / powerToMassRatio
        mass_check = electrical_power / powerToMassRatio

        outputs["converter_weight"] = mass_inv_conv
        outputs["data:geometry:propulsion:converter:weight"] = mass_inv_conv
