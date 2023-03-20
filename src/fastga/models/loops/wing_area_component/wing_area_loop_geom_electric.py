import logging
import warnings

import numpy as np
import openmdao.api as om

import fastoad.api as oad

from ..constants import SUBMODEL_WING_AREA_GEOM_LOOP, SUBMODEL_WING_AREA_GEOM_CONS

_LOGGER = logging.getLogger(__name__)


@oad.RegisterSubmodel(
    SUBMODEL_WING_AREA_GEOM_LOOP, "fastga.submodel.loop.wing_area.update.geom.electric"
)
class UpdateWingAreaGeomElectric(om.ExplicitComponent):

    def setup(self):
        self.add_input("data:geometry:wing:root:chord", val=np.nan, units="m")
        self.add_input("data:geometry:wing:tip:chord", val=np.nan, units="m")
        self.add_input("data:geometry:wing:root:thickness_ratio", val=np.nan)
        self.add_input("data:geometry:wing:tip:thickness_ratio", val=np.nan)
        self.add_input("data:geometry:propulsion:battery:volume", val=np.nan, units="m**3")

        self.add_output("wing_area", val=10.0, units="m**2")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        root_chord = inputs["data:geometry:wing:root:chord"]
        tip_chord = inputs["data:geometry:wing:tip:chord"]
        root_thickness_ratio = inputs["data:geometry:wing:root:thickness_ratio"]
        tip_thickness_ratio = inputs["data:geometry:wing:tip:thickness_ratio"]
        battery_volume = inputs["data:geometry:propulsion:battery:volume"]

        ave_thickness = (
                0.7 * (root_chord * root_thickness_ratio + tip_chord * tip_thickness_ratio) / 2.0
        )

        wing_area_mission = battery_volume / (0.3 * ave_thickness)

        _LOGGER.debug("Wing area electric")

        outputs["wing_area"] = wing_area_mission
