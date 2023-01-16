import fastoad.api as oad
import openmdao.api as om
import numpy as np

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_SWITCH_MASS


@oad.RegisterSubmodel(
    SUBMODEL_SWITCH_MASS,
    "fastga.submodel.propulsion.hybrid_propulsion.switch.legacy",
)
class ComputeSwitchMass(om.ExplicitComponent):
    """Estimation on switch mass [kg] based on maximum power required from motor
           Based on:
        """
    def initialize(self):
        self.options.declare(
            "number_of_points", default=252, desc="number of equilibrium to be treated"
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_input("mechanical_power", shape=number_of_points)
        self.add_input("data:mission:sizing:takeoff:power", np.nan)
        self.add_input("motor_efficiency", val=0.85)
        self.add_input("switch_efficiency", val=0.95)
        self.add_input("gearbox_efficiency", val=0.98)
        self.add_input("controller_efficiency", val=0.95)
        self.add_output(
            "switch_weight",
            val=50.00,
            units="kg",
            desc="switch mass",
        )

        self.add_output("data:geometry:propulsion:switch:weight", units="kg", desc="switch mass")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        total_efficiency = inputs["motor_efficiency"] * inputs["gearbox_efficiency"] * inputs["controller_efficiency"] \
                           * inputs["switch_efficiency"]

        mechanical_power_total = np.append(np.array(inputs["data:mission:sizing:takeoff:power"]),
                                           inputs["mechanical_power"])
        max_power = max(mechanical_power_total)
        electrical_power = max_power/total_efficiency
        mass_switch = 1.6 * 10 ** -4 * electrical_power + 0.6
        outputs["switch_weight"] = mass_switch
        outputs["data:geometry:propulsion:switch:weight"] = mass_switch
