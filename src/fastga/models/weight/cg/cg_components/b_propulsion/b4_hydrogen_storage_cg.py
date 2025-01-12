"""Estimation of hydrogen storage system center of gravity."""
#  This file is part of FAST-OAD_CS23 : A framework for rapid Overall Aircraft Design
#  Copyright (C) 2022  ONERA & ISAE-SUPAERO
#  FAST is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

import numpy as np
from openmdao.core.explicitcomponent import ExplicitComponent


class ComputeHydrogenStorageCG(ExplicitComponent):
    # TODO: Document equations. Cite sources
    """Hydrogen storage center of gravity estimation"""

    def setup(self):

        self.add_input("data:weight:propulsion:fuselage:H2_storage_mass", val=np.nan, units="kg")
        self.add_input("data:geometry:fuselage:length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:tank_length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:rear_length", val=np.nan, units="m")

        self.add_output("data:weight:propulsion:H2_storage:CG:x", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        cg_b4 = inputs["data:geometry:fuselage:length"] - 0.5*inputs["data:geometry:fuselage:tank_length"] - inputs["data:geometry:fuselage:rear_length"]

        outputs["data:weight:propulsion:H2_storage:CG:x"] = cg_b4

class ComputeHydrogenStorageDorsalCG(ExplicitComponent):
    # TODO: Document equations. Cite sources
    """Hydrogen storage center of gravity estimation"""

    def setup(self):

        self.add_input("data:weight:propulsion:fuselage:H2_storage_mass", val=np.nan, units="kg")
        self.add_input("data:geometry:fuselage:length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:tank_length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:front_length", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:length", val=np.nan, units="m")

        self.add_output("data:weight:propulsion:H2_storage:CG:x", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        cg_b4 = inputs["data:geometry:fuselage:front_length"] + 0.5*inputs["data:geometry:cabin:length"]

        outputs["data:weight:propulsion:H2_storage:CG:x"] = cg_b4


class ComputeHydrogenStorageForwardCG(ExplicitComponent):
    # TODO: Document equations. Cite sources
    """Hydrogen storage center of gravity estimation"""

    def setup(self):

        self.add_input("data:weight:propulsion:fuselage:H2_storage_mass", val=np.nan, units="kg")
        self.add_input("data:geometry:fuselage:length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:tank_length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:front_length", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:length", val=np.nan, units="m")

        self.add_output("data:weight:propulsion:H2_storage:CG:x", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        cg_b4 = inputs["data:geometry:fuselage:front_length"] + 0.1*inputs["data:geometry:cabin:length"] + 0.5*inputs["data:geometry:fuselage:tank_length"]

        outputs["data:weight:propulsion:H2_storage:CG:x"] = cg_b4