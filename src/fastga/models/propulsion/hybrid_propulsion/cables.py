import logging
import fastoad.api as oad
import openmdao.api as om

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_CABLES_MASS
_LOGGER = logging.getLogger(__name__)


@oad.RegisterSubmodel(SUBMODEL_CABLES_MASS,
                      "fastga.submodel.propulsion.hybrid_propulsion.cables.legacy")
class ComputeCableMass(om.ExplicitComponent):
    def setup(self):
        self.add_input("data:propulsion:motor:efficiency", val=0.93)
        self.add_input("data:propulsion:battery:efficiency", val=0.93)
        self.add_input("data:propulsion:switch:efficiency", val=0.97)
        self.add_input("data:propulsion:gearbox:efficiency", val=0.98)
        self.add_input("data:propulsion:controller:efficiency", val=0.97)
        self.add_input("data:propulsion:bus:efficiency", val=0.97)
        self.add_input("data:propulsion:converter:efficiency", val=0.97)
        self.add_input("data:propulsion:cables:efficiency", val=0.99)

        self.add_output("cables_weight")
        self.add_output("data:geometry:propulsion:cables:weight", units="kg")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        _LOGGER.debug("Calculating cable parameters")
        outputs["cables_weight"] = 500  # 15% of the total power train mass
        outputs["data:geometry:propulsion:cables:weight"] = 500  # in [kg]

