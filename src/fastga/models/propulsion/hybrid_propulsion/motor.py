import fastoad.api as oad
import numpy as np
import openmdao.api as om
from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_MOTOR_MASS


@oad.RegisterSubmodel(
    SUBMODEL_MOTOR_MASS,
    "fastga.submodel.propulsion.hybrid_propulsion.motor.legacy",
)
class ComputeMotorMass(om.ExplicitComponent):
    """Estimation on motor mass [kg] based on maximum power required from motor
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
        self.add_input("data:propulsion:motor:number", val=8, desc="number of motors")  #### check input
        self.add_output(
            "motor_weight",
            val=np.nan,
            units="kg",
            desc="motor mass of all motors",
        )
        self.add_output("data:geometry:propulsion:motor:weight", units="kg", desc="motor mass")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        mechanical_power_total = np.append(np.array(inputs["data:mission:sizing:takeoff:power"]),
                                           inputs["mechanical_power"])
        max_power = max(mechanical_power_total)
        electrical_power = max_power / inputs["motor_efficiency"] / inputs["data:propulsion:motor:number"]
        mass_motor = (0.095 * electrical_power + 30) * inputs["data:propulsion:motor:number"]

        outputs["motor_weight"] = mass_motor
        outputs["data:geometry:propulsion:motor:weight"] = mass_motor
