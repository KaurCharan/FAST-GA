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
import fastoad.api as oad

from ..constants import SUBMODEL_EQUILIBRIUM
from ..mission.compute_time_step import ComputeTimeStep
from ..mission.performance_per_phase import PerformancePerPhase
from ..mission.reserve_energy import ReserveEnergy
from ..mission.sizing_energy import SizingEnergy
from ..mission.thrust_taxi import ThrustTaxi
from ..mission.update_mass import UpdateMass


class MissionCore(om.Group):
    """Find the conditions necessary for the aircraft equilibrium."""

    def initialize(self):
        self.options.declare(
            "number_of_points", default=1, desc="number of equilibrium to be treated"
        )
        self.options.declare("propulsion_id", default=None, types=str, allow_none=True)

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_subsystem(
            "compute_taxi_thrust",
            ThrustTaxi(propulsion_id=self.options["propulsion_id"]),
            promotes=["*"],
        )
        self.add_subsystem(
            "compute_time_step",
            ComputeTimeStep(number_of_points=number_of_points),
            promotes_inputs=["time"],
            promotes_outputs=["time_step"],
        )
        options_equilibrium = {
            "number_of_points": number_of_points,
            "propulsion_id": self.options["propulsion_id"],
            "promotes_all_variables": True
        }
        self.add_subsystem(
            "compute_dep_equilibrium",
            oad.RegisterSubmodel.get_submodel(SUBMODEL_EQUILIBRIUM, options=options_equilibrium),
            promotes=["*"]
        )
        self.add_subsystem(
            "performance_per_phase",
            PerformancePerPhase(number_of_points=number_of_points),
            promotes_inputs=["fuel_consumed_t_econ", "position", "time", "non_consumable_energy_t_econ", "thrust_rate_t_econ"],
            promotes_outputs=["data:*", "fuel_consumed_t", "non_consumable_energy_t", "thrust_rate_t"],
        )
        self.add_subsystem("reserve_fuel", ReserveEnergy(), promotes=["*"])
        self.add_subsystem("sizing_fuel", SizingEnergy(), promotes=["*"])
        self.add_subsystem(
            "update_mass",
            UpdateMass(number_of_points=number_of_points),
            promotes_inputs=["data:*", "fuel_consumed_t"],
            promotes_outputs=["mass"],
        )

