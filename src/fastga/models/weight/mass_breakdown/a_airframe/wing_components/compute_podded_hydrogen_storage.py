"""
Computes the mass of podded hydrogen storage below the wing
"""
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

import openmdao.api as om
import numpy as np

class ComputePoddedHydrogenStorage(om.ExplicitComponent):
    def setup(self):
        self.add_input("data:geometry:fuselage:H2_storage_mass")
        self.add_input("data:geometry:wing:span", val=np.nan, units="m")

        self.add_output(
            "data:weight:airframe:wing:punctual_mass:y_ratio",
            shape_by_conn=False,
            val=0.0,
        )
        self.add_output(
            "data:weight:airframe:wing:punctual_mass:mass",
            shape_by_conn=False,
            copy_shape="data:weight:airframe:wing:punctual_mass:y_ratio",
            units="kg",
            val=0.0,
        )
    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        H2_tank_mass = inputs["data:geometry:fuselage:H2_storage_mass"]
        H2_podded_mass = np.array([H2_tank_mass/2])
        H2_podded_position = np.array([0.8])
        print(H2_podded_mass)
        outputs["data:weight:airframe:wing:punctual_mass:mass"] = H2_podded_mass
        outputs["data:weight:airframe:wing:punctual_mass:y_ratio"] = H2_podded_position


        # self.add_output("data:weight:airframe:wing:hydrogen_storage:mass")
        # self.add_output("data:weight:airframe:wing:hydrogen_storage:position")
        # wing_span = inputs["data:geometry:wing:span"]