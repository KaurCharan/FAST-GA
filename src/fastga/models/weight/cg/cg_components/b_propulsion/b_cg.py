"""Estimation of center of gravity for a fuel propulsion system."""
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

import fastoad.api as oad

from .b1_engine_cg import ComputeEngineCG
from .b2_fuel_lines_cg import ComputeFuelLinesCG
from .b4_hydrogen_storage_cg import ComputeHydrogenStorageCG #additional code
from .b5_fuelcell_cg import ComputeFuelCellCG #additional code

from ..constants import SUBMODEL_PROPULSION_CG

@oad.RegisterSubmodel(SUBMODEL_PROPULSION_CG, "fastga.submodel.weight.cg.propulsion.legacy")
class FuelPropulsionCG(om.Group):
    def setup(self):
        self.add_subsystem("engine_cg", ComputeEngineCG(), promotes=["*"])
        self.add_subsystem("fuel_lines_cg", ComputeFuelLinesCG(), promotes=["*"])
        self.add_subsystem("propulsion_cg", ComputeFuelPropulsionCG(), promotes=["*"])

class ComputeFuelPropulsionCG(om.ExplicitComponent):
    def setup(self):
        self.add_input("data:weight:propulsion:engine:CG:x", units="m", val=np.nan)
        self.add_input("data:weight:propulsion:fuel_lines:CG:x", units="m", val=np.nan)

        self.add_input("data:weight:propulsion:engine:mass", units="kg", val=np.nan)
        self.add_input("data:weight:propulsion:fuel_lines:mass", units="kg", val=np.nan)
        self.add_input("data:weight:propulsion:mass", units="kg", val=np.nan)

        self.add_output("data:weight:propulsion:CG:x", units="m")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        engine_cg = inputs["data:weight:propulsion:engine:CG:x"]
        fuel_lines_cg = inputs["data:weight:propulsion:fuel_lines:CG:x"]

        engine_mass = inputs["data:weight:propulsion:engine:mass"]
        fuel_lines_mass = inputs["data:weight:propulsion:fuel_lines:mass"]

        cg_propulsion = (engine_cg * engine_mass + fuel_lines_cg * fuel_lines_mass) / (
            engine_mass + fuel_lines_mass
        )

        outputs["data:weight:propulsion:CG:x"] = cg_propulsion

@oad.RegisterSubmodel(SUBMODEL_PROPULSION_CG, "fastga.submodel.weight.cg.propulsion.hydrogenrear")
class FuelPropulsionHydrogenRearCG(om.Group):
    def setup(self):
        self.add_subsystem("engine_cg", ComputeEngineCG(), promotes=["*"])
        self.add_subsystem("fuel_lines_cg", ComputeFuelLinesCG(), promotes=["*"])
        self.add_subsystem("propulsion_cg", ComputeFuelPropulsionHydrogenFuelCG(), promotes=["*"])
        self.add_subsystem("hydrogen_storage_cg", ComputeHydrogenStorageCG(), promotes=["*"]) #additional code
        self.add_subsystem("fuelcell_cg", ComputeFuelCellCG(), promotes=["*"]) #additional code

class ComputeFuelPropulsionHydrogenFuelCG(om.ExplicitComponent):
    def setup(self):
        self.add_input("data:weight:propulsion:engine:CG:x", units="m", val=np.nan)
        self.add_input("data:weight:propulsion:fuel_lines:CG:x", units="m", val=np.nan)
        self.add_input("data:weight:propulsion:H2_storage:CG:x", units="m", val=np.nan) #additional code
        # self.add_input("data:weight:propulsion:H2_storage:CG:x", units="m", val=np.nan) #additional code

        self.add_input("data:weight:propulsion:engine:mass", units="kg", val=np.nan)
        self.add_input("data:weight:propulsion:fuel_lines:mass", units="kg", val=np.nan)
        self.add_input("data:weight:propulsion:mass", units="kg", val=np.nan)
        self.add_input("data:weight:propulsion:fuselage:H2_storage_mass", units="kg", val=np.nan) #additional code
        self.add_input("data:weight:propulsion:fuselage:fuelcell", units="kg", val=np.nan) #additional code

        self.add_output("data:weight:propulsion:CG:x", units="m")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        engine_cg = inputs["data:weight:propulsion:engine:CG:x"]
        fuel_lines_cg = inputs["data:weight:propulsion:fuel_lines:CG:x"]
        h2_storage_cg = inputs["data:weight:propulsion:H2_storage:CG:x"] #additional code
        fuelcell_cg = inputs["data:weight:propulsion:H2_storage:CG:x"] #additional code

        engine_mass = inputs["data:weight:propulsion:engine:mass"]
        fuel_lines_mass = inputs["data:weight:propulsion:fuel_lines:mass"]
        h2_storage_mass = inputs["data:weight:propulsion:fuselage:H2_storage_mass"] #additional code
        fuelcell_mass = inputs["data:weight:propulsion:fuselage:fuelcell"] #additional code

        cg_propulsion = (engine_cg * engine_mass + fuel_lines_cg * fuel_lines_mass + h2_storage_cg * h2_storage_mass + fuelcell_cg * fuelcell_mass) / (
            engine_mass + fuel_lines_mass + h2_storage_mass + fuelcell_mass
        ) #modified code

        outputs["data:weight:propulsion:CG:x"] = cg_propulsion
