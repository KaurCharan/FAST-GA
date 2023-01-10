import fastoad.api as oad
import openmdao.api as om

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_SUMMATION_WEIGHTS


@oad.RegisterSubmodel(SUBMODEL_SUMMATION_WEIGHTS,
                      "fastga.submodel.propulsion.hybrid_propulsion.summation.legacy")
class SummationWeights(om.ExplicitComponent):

    def setup(self):
        self.add_input("fuelcell_weight")
        self.add_input("converter_weight")
        self.add_input("battery_weight")
        self.add_input("switch_weight")
        self.add_input("motor_weight")
        self.add_input("cables_weight")

        self.add_input("data:propulsion:controller:number")

        self.add_output("data:propulsion:total_weight")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        summation = inputs["fuelcell_weight"] + inputs["converter_weight"] + inputs["battery_weight"] + \
                    inputs["switch_weight"] + inputs["motor_weight"] + inputs["cables_weight"]

        outputs["data:propulsion:total_weight"] = summation + 1 * inputs["data:propulsion:controller:number"]
