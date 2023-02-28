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

import logging
import numpy as np
import openmdao.api as om

# noinspection PyProtectedMember
import fastoad.api as oad

from fastga.models.propulsion.hybrid_propulsion.battery import BatteryModel
from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_BATTERY_PARAMETERS

_LOGGER = logging.getLogger(__name__)


@oad.RegisterSubmodel(SUBMODEL_BATTERY_PARAMETERS,
                      "fastga.submodel.propulsion.hybrid_propulsion.battery.legacy")
class BatteryParameters(om.ExplicitComponent):
    """Computes weight and soc at each time step."""

    def initialize(self):
        self.options.declare(
            "number_of_points", default=252, desc="number of equilibrium to be treated"
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_input("mechanical_power", units="W", shape=number_of_points)
        self.add_input("data:mission:sizing:takeoff:power", units="W")
        self.add_input("data:mission:sizing:takeoff:duration", units="s")
        self.add_input("fuelcell_Pelec_max", units="W")
        self.add_input("motor_efficiency", val=0.93)
        self.add_input("battery_efficiency", val=0.98)
        self.add_input("switch_efficiency", val=0.97)
        self.add_input("gearbox_efficiency", val=0.98)
        self.add_input("controller_efficiency", val=0.97)
        self.add_input("bus_efficiency", val=0.97)
        self.add_input("converter_efficiency", val=0.97)
        self.add_input("cables_efficiency", val=0.99)

        self.add_input(
            "time_step_econ",
            shape=number_of_points,
            val=np.full(number_of_points, 0.1),
            units="s",
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
            "battery_weight",
            val=980.00,
            units="kg",
            desc="mass of all battery packs",
        )
        self.add_output(
            "volume_BatteryPack",
            val=0.6,
            units="m**3",
            desc="volume of all battery packs",
        )

        self.add_output(
            "battery_efficiency_out",
            val=np.full(101, 0.5),
            desc="efficiency of battery",
        )

        self.add_output("non_consumable_energy_t_econ", val=0, shape=number_of_points, units="W*h")
        self.add_output("data:geometry:propulsion:battery:volume", units="m**3", desc="volume of battery pack")
        self.add_output("data:geometry:propulsion:battery:weight", units="kg", desc="mass of battery pack")
        self.add_output("data:propulsion:battery:dod", val=0, shape=101)
        self.add_output("data:geometry:propulsion:battery:n_parallel")
        self.add_output("data:geometry:propulsion:battery:n_series")
        self.add_output("data:propulsion:battery:efficiency_out", val=0, shape=101)

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        _LOGGER.debug("Calculating battery parameters")
        total_efficiency = inputs["motor_efficiency"] * inputs["gearbox_efficiency"] * inputs["controller_efficiency"] \
                           * inputs["switch_efficiency"] * inputs["bus_efficiency"] * inputs["converter_efficiency"] * \
                           inputs["cables_efficiency"] * inputs["battery_efficiency"]

        fuelcell_Pelec_max = inputs["fuelcell_Pelec_max"]
        time_step = inputs["time_step_econ"]
        time_TO = np.array(inputs["data:mission:sizing:takeoff:duration"])
        mechanical_power = inputs["mechanical_power"]
        mechanical_power_TO = np.array(inputs["data:mission:sizing:takeoff:power"])
        mechanical_power_climb = mechanical_power[0:100]
        electrical_power = list(range(len(mechanical_power_climb)))
        electrical_power_climb = mechanical_power_climb / total_efficiency
        weight_cells=0
        for idx in range(np.size(electrical_power_climb)):
            if electrical_power_climb[idx] > fuelcell_Pelec_max:
                electrical_power[idx] = float(abs(electrical_power_climb[idx] - fuelcell_Pelec_max))
            else:
                electrical_power[idx] = 0
        if (mechanical_power_TO / total_efficiency / 0.80) > fuelcell_Pelec_max:
            electrical_power_TO = float((mechanical_power_TO / total_efficiency / 0.80) - fuelcell_Pelec_max)
        else:
            electrical_power_TO = 0

        # append power and time together
        electrical_power_total = np.append(electrical_power_TO, electrical_power)
        time_total = np.append(time_TO, time_step[0:100])
        if all(electrical_power_total <= 0) or (any(ele > 3e6 for ele in electrical_power_total) == 1):
            energy_consumed = np.zeros(252)
            dod = np.zeros(101)
            eff_bat = np.zeros(101)
            volume_BatteryPack = 0.6
            weight_BatteryPack = 980
            n_series = 0
            n_parallel = 0
            _LOGGER.debug("Skipped battery model")
        else:
            battery_model = BatteryModel(electrical_power_total, time_total)

            # storing parameters
            weight_cells, n_series, n_parallel, dod, eff_bat, Q_used= battery_model.compute_soc()
            weight_BatteryPack = battery_model.compute_weight(weight_cells)
            volume_BatteryPack = battery_model.compute_volume(n_parallel, n_series) / 10 ** 9   # in m3
            extra = [0.0 for i in range(152)]
            #energy_consumed = np.concatenate((Q_used[1:101], extra))
            energy_consumed = np.zeros(252)
        print("total battery cells weight is", weight_cells)
        print("total battery pack weight is", weight_BatteryPack)
        print("number of modules in parallel", n_parallel)
        outputs["non_consumable_energy_t_econ"] = energy_consumed
        outputs["data:propulsion:battery:dod"] = dod
        outputs["data:geometry:propulsion:battery:n_parallel"] = n_parallel
        outputs["data:geometry:propulsion:battery:n_series"] = n_series
        outputs["data:geometry:propulsion:battery:weight"] = weight_BatteryPack # [in kg]
        outputs["battery_weight"] = weight_BatteryPack
        outputs["data:geometry:propulsion:battery:volume"] = volume_BatteryPack
        outputs["data:propulsion:battery:efficiency_out"] = eff_bat
        print("hehe")
