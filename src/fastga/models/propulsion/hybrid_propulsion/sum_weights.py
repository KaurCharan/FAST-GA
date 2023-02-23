import logging
import fastoad.api as oad
import openmdao.api as om

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_SUMMATION_WEIGHTS

_LOGGER = logging.getLogger(__name__)


@oad.RegisterSubmodel(SUBMODEL_SUMMATION_WEIGHTS,
                      "fastga.submodel.propulsion.hybrid_propulsion.summation.legacy")
class SummationWeights(om.ExplicitComponent):
    def initialize(self):
        self.options.declare(
            "number_of_points", default=252, desc="number of equilibrium to be treated"
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]
        self.add_input("mechanical_power", units="W", shape=number_of_points)
        self.add_input("fuelcell_weight", units="kg")
        self.add_input("converter_weight", units="kg")
        self.add_input("battery_weight", units="kg")
        self.add_input("switch_weight", units="kg")
        self.add_input("motor_weight", units="kg")
        self.add_input("cables_weight", units="kg")

        self.add_input("data:propulsion:controller:number")

        self.add_output("data:propulsion:total_weight", units="kg", val=2000)

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        _LOGGER.debug("Calculating total power plant weight")
        summation = inputs["fuelcell_weight"] + inputs["converter_weight"] + inputs["battery_weight"] + \
                    inputs["switch_weight"] + inputs["motor_weight"] + inputs["cables_weight"] + \
                    1 * inputs["data:propulsion:controller:number"]
        power = inputs["mechanical_power"]
        outputs["data:propulsion:total_weight"] = summation
