"""Estimation of the hydrogen storage profile drag."""
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

import math

import numpy as np
import fastoad.api as oad
from openmdao.core.explicitcomponent import ExplicitComponent

from ..constants import SUBMODEL_CD0_HYDROGEN_STORAGE

@oad.RegisterSubmodel(SUBMODEL_CD0_HYDROGEN_STORAGE, "fastga.submodel.aerodynamics.hydrogen_storage.cd0")
class Cd0HydrogenStorage(ExplicitComponent):
    """
    Profile drag estimation for the hydrogen storage podded below the wing

    Based on : Gudmundsson, Snorri. General aviation aircraft design: Applied Methods and
    Procedures. Butterworth-Heinemann, 2013.
    """

    def initialize(self):
        self.options.declare("low_speed_aero", default=False, types=bool)

    def setup(self):
        self.add_input("data:geometry:wing:area", val=np.nan, units="m**2")
        self.add_input("data:geometry:fuselage:a", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:b", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:tank_length", val=np.nan, units="m")

        if self.options["low_speed_aero"]:
            self.add_output("data:aerodynamics:hydrogen_storage:low_speed:CD0")
        else:
            self.add_output("data:aerodynamics:hydrogen_storage:cruise:CD0")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        wing_area = inputs["data:geometry:wing:area"]
        a = inputs["data:geometry:fuselage:a"]
        b = inputs["data:geometry:fuselage:b"]
        tank_length = inputs["data:geometry:fuselage:tank_length"]

        wet_area = 2*math.pi*a*tank_length + 4*math.pi*a*b
        cds_hydrogen_storage = 0.10
        cd0_hydrogen_storage = 2*cds_hydrogen_storage*wet_area/wing_area

        if self.options["low_speed_aero"]:
            outputs["data:aerodynamics:hydrogen_storage:low_speed:CD0"] = cd0_hydrogen_storage
        else:
            outputs["data:aerodynamics:hydrogen_storage:cruise:CD0"] = cd0_hydrogen_storage