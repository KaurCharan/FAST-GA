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
import openmdao.api as om

# noinspection PyProtectedMember
import fastoad.api as oad

from fastga.models.propulsion.hybrid_propulsion.battery import BatteryModel
from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_BATTERY_PARAMETERS


@oad.RegisterSubmodel(SUBMODEL_BATTERY_PARAMETERS,
                      "fastga.submodel.propulsion.hybrid_propulsion.battery.legacy")
class BatteryParameters(om.ExplicitComponent):
    """Computes weight and soc at each time step."""

    def initialize(self):
        self.options.declare(
            "number_of_points", default=1, desc="number of equilibrium to be treated"
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_input("mechanical_power")
        self.add_input("data:mission:sizing:takeoff:power", np.nan)  #### check
        self.add_input("data:mission:sizing:takeoff:duration")
        self.add_input("fuelcell_Pelec_max")
        self.add_input("motor_efficiency", val=0.85)
        self.add_input("battery_efficiency", val=0.95)
        self.add_input("switch_efficiency", val=0.95)
        self.add_input("gearbox_efficiency", val=0.98)
        self.add_input("controller_efficiency", val=0.95)
        self.add_input("bus_efficiency", val=0.95)
        self.add_input("converter_efficiency", val=0.90)
        self.add_input("cables_efficiency", val=0.80)

        self.add_input(
            "time_step",
            shape=number_of_points + 2,
            val=np.full(number_of_points + 2, np.nan),
            units="s",
        )

        self.add_output(
            "soc",
            val=np.full(number_of_points + 2, 0.0),
            desc="soc at each time step",
            units="percent",
        )
        self.add_output(
            "n_parallel",
            shape=1,
            val=222,
            desc="number of cells in parallel",
        )
        self.add_output(
            "n_series",
            shape=1,
            val=139,
            desc="number of cells in series",
        )
        self.add_output(
            "weight_BatteryPack",
            val=1925.00,
            units="kg",
            desc="mass of all battery packs",
        )
        self.add_output(
            "volume_BatteryPack",
            val=2000.00,
            units="m**3",
            desc="volume of all battery packs",
        )

        self.add_output(
            "battery_efficiency_out",
            val=np.full(number_of_points + 2, 0.5),
            desc="efficiency of battery",
        )

        self.add_output("data:geometry:propulsion:battery:volume", units="m**3", desc="volume of battery pack")
        self.add_output("data:geometry:propulsion:battery:weight", units="kg", desc="mass of battery pack")
        self.add_output("data:propulsion:battery:soc")
        self.add_output("data:geometry:propulsion:battery:n_parallel")
        self.add_output("data:geometry:propulsion:battery:n_series")
        self.add_output("data:propulsion:battery:efficiency_out")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        total_efficiency = inputs["motor_efficiency"] * inputs["gearbox_efficiency"] * inputs["controller_efficiency"] \
                           * inputs["switch_efficiency"] * inputs["bus_efficiency"] * inputs["converter_efficiency"] * \
                           inputs["cables_efficiency"] * inputs["battery_efficiency"]

        fuelcell_Pelec_max = inputs["fuelcell_Pelec_max"]
        time_step = inputs["time_step"]
        time_TO = np.array(inputs["data:mission:sizing:takeoff:duration"])
        mechanical_power = inputs["mechanical_power"]
        mechanical_power_TO = np.array(inputs["data:mission:sizing:takeoff:power"])

        electrical_power = (mechanical_power[0:99] / total_efficiency) - fuelcell_Pelec_max
        electrical_power_TO = (mechanical_power_TO / total_efficiency) - fuelcell_Pelec_max

        # append power and time together
        electrical_power_total = np.append(electrical_power_TO, electrical_power)
        time_total = np.append(time_TO, time_step)
        battery_model = BatteryModel(electrical_power_total, time_total)

        # storing parameters
        weight_cells, n_series, n_parallel, soc, eff_bat, C_rate = battery_model.compute_soc()
        weight_BatteryPack = battery_model.compute_weight(weight_cells)
        volume_BatteryPack = battery_model.compute_volume(n_parallel, n_series) / 10 ** 9   # in m3

        outputs["data:propulsion:battery:soc"] = soc
        outputs["data:geometry:propulsion:battery:n_parallel"] = n_parallel
        outputs["data:geometry:propulsion:battery:n_series"] = n_series
        outputs["data:geometry:propulsion:battery:weight"] = weight_BatteryPack  # [in kg]
        outputs["battery_weight"] = weight_BatteryPack
        outputs["data:geometry:propulsion:battery:volume"] = volume_BatteryPack
        outputs["data:propulsion:battery:efficiency_out"] = eff_bat
