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
        self.add_input("data:propulsion:system_voltage", units="V")
        self.add_input("fuelcell_Pelec_max", units="W")
        self.add_input("data:propulsion:motor:efficiency", val=0.93)
        self.add_input("battery_efficiency", val=0.93)
        self.add_input("data:propulsion:switch:efficiency", val=0.97)
        self.add_input("data:propulsion:gearbox:efficiency", val=0.98)
        self.add_input("data:propulsion:controller:efficiency", val=0.97)
        self.add_input("data:propulsion:bus:efficiency", val=0.97)
        self.add_input("data:propulsion:converter:efficiency", val=0.97)
        self.add_input("data:propulsion:cables:efficiency", val=0.99)

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
        self.add_output("data:propulsion:battery:efficiency", val=0.93, shape=1)

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        _LOGGER.debug("Calculating battery parameters")
        total_efficiency = inputs["data:propulsion:motor:efficiency"] * inputs["data:propulsion:gearbox:efficiency"] \
                           * inputs["data:propulsion:controller:efficiency"] \
                           * inputs["data:propulsion:switch:efficiency"] * inputs["data:propulsion:bus:efficiency"] \
                           * inputs["data:propulsion:converter:efficiency"] \
                           * inputs["data:propulsion:cables:efficiency"] * inputs["battery_efficiency"]

        voltage = inputs["data:propulsion:system_voltage"]
        fuelcell_Pelec_max = inputs["fuelcell_Pelec_max"]
        time_step = inputs["time_step_econ"]
        time_TO = np.array(inputs["data:mission:sizing:takeoff:duration"])
        mechanical_power = inputs["mechanical_power"]
        mechanical_power_TO = np.array(inputs["data:mission:sizing:takeoff:power"])

        mechanical_power_climb = mechanical_power[0:100]  # stores flight points for climb
        # initialises for electrical power requirement from battery
        electrical_power = list(range(len(mechanical_power_climb)))
        electrical_power_climb = mechanical_power_climb / total_efficiency  # total electrical power for climb points

        # Subtracts electrical power supplied by fuelcell from total power requirement at flight points for takeoff
        # and climb. This gives the battery power requirement as per scheme 3. Scheme 3 means that fuel cell operates
        # during the cruise and landing phases. During takeoff and climb, fuel cell supplies power equivalent to its
        # maximum power and the remaining is supplied by the battery. Hence, the battery is sized only for takeoff
        # and climb.
        for idx in range(np.size(electrical_power_climb)):
            if electrical_power_climb[idx] > fuelcell_Pelec_max:
                # electrical power requirement from battery for climb
                electrical_power[idx] = float(abs(electrical_power_climb[idx] - fuelcell_Pelec_max))
            else:
                electrical_power[idx] = 0
        if (mechanical_power_TO / total_efficiency / 0.80) > fuelcell_Pelec_max:
            # electrical power requirement from battery for takeoff.
            # here, 80% propeller efficiency is assumed. Note that the input takeoff power is aerodynamic power.
            electrical_power_TO = float((mechanical_power_TO / total_efficiency / 0.80) - fuelcell_Pelec_max)
        else:
            electrical_power_TO = 0

        # append battery power requirement and time together
        electrical_power_total = np.append(electrical_power_TO, electrical_power)
        time_total = np.append(time_TO, time_step[0:100])

        # During code execution, for some iterations, it is possible that fuelcell power is greater than takeoff and
        # climb power so, power requirement from battery will become zero. In this case, the battery model is skipped.
        if all(electrical_power_total <= 0):
            energy_consumed = np.zeros(252)
            dod = np.zeros(101)
            eff_bat = np.zeros(101)
            volume_BatteryPack = 0.6
            weight_BatteryPack = 980
            n_series = 0
            n_parallel = 0
            battery_eff_avg = 0.93
            _LOGGER.debug("Skipped battery model")
        else:
            # battery power requirement, time and bus voltage are inputs to the battery model.
            battery_model = BatteryModel(electrical_power_total, time_total, voltage)

            # storing parameters
            # call soc function in battery model.
            weight_cells, n_series, n_parallel, dod, eff_bat, Q_used = battery_model.compute_soc()
            # call weight function in battery model.
            weight_BatteryPack = battery_model.compute_weight(weight_cells)
            # call volume function in battery model.
            volume_BatteryPack = battery_model.compute_volume(n_parallel, n_series) / 10 ** 9  # in [m3]
            # calculates an average battery efficiency.
            battery_eff_avg = np.average(eff_bat)
            # here, it is necessary to output non_consumable_energy_t_econ because of performance_per_phase.py needs
            # it. Zeros are given as output for now as it does not impact the sizing.
            energy_consumed = np.zeros(252)
        outputs["non_consumable_energy_t_econ"] = energy_consumed  # in [Ah]
        outputs["data:propulsion:battery:dod"] = dod  # in [%]
        outputs["data:geometry:propulsion:battery:n_parallel"] = n_parallel
        outputs["data:geometry:propulsion:battery:n_series"] = n_series
        outputs["data:geometry:propulsion:battery:weight"] = weight_BatteryPack  # in [kg]
        outputs["battery_weight"] = weight_BatteryPack  # in [kg]
        outputs["data:geometry:propulsion:battery:volume"] = volume_BatteryPack  # in [m3]
        outputs["data:propulsion:battery:efficiency_out"] = eff_bat  # in [%]
        outputs["data:propulsion:battery:efficiency"] = battery_eff_avg  # in [%]
