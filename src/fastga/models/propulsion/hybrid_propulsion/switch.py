import logging
import fastoad.api as oad
import openmdao.api as om
import numpy as np

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_SWITCH_MASS

_LOGGER = logging.getLogger(__name__)


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

        self.add_input("mechanical_power", units="W", shape=number_of_points)
        self.add_input("data:mission:sizing:takeoff:power", units="W")
        self.add_input("data:propulsion:motor:efficiency", val=0.93)
        self.add_input("data:propulsion:switch:efficiency", val=0.97)
        self.add_input("data:propulsion:gearbox:efficiency", val=0.98)
        self.add_input("data:propulsion:controller:efficiency", val=0.97)

        self.add_output(
            "switch_weight",
            val=50.00,
            units="kg",
            desc="switch mass",
        )

        self.add_output("data:geometry:propulsion:switch:weight", units="kg", desc="switch mass")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        _LOGGER.debug("Calculating switch parameters")
        total_efficiency = inputs["data:propulsion:motor:efficiency"] * inputs["data:propulsion:gearbox:efficiency"] \
                           * inputs["data:propulsion:controller:efficiency"] * inputs[
                               "data:propulsion:switch:efficiency"]

        mechanical_power_total = np.append(np.array(inputs["data:mission:sizing:takeoff:power"]),
                                           inputs["mechanical_power"])
        max_power = max(mechanical_power_total) / 1000  # in [kW]
        electrical_power = max_power / total_efficiency
        if any(ele > 1e6 for ele in electrical_power) == 1:
            mass_switch = 1
        else:
            mass_switch = 1.6 * 1e-4 * electrical_power + 0.6

        outputs["switch_weight"] = mass_switch
        outputs["data:geometry:propulsion:switch:weight"] = mass_switch
