"""Parametric propeller IC engine."""
# -*- coding: utf-8 -*-
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

from typing import Union, Sequence, Tuple, Optional
from scipy.interpolate import interp2d
import numpy as np
import fastoad.api as oad
import openmdao.api as om

from stdatm import Atmosphere
from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_COMPUTE_POWER
from fastga.models.aerodynamics.external.propeller_code.compute_propeller_aero import (
    THRUST_PTS_NB,
    SPEED_PTS_NB
)


@oad.RegisterSubmodel(
    SUBMODEL_COMPUTE_POWER,
    "fastga.submodel.propulsion.hybrid_propulsion.power.legacy",
)
class ComputePower(om.ExplicitComponent):

    def initialize(self):

        self.options.declare(
            "number_of_points", default=252, desc="number of flight points"
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_input("thrust_econ", val=np.full(number_of_points, 0.0), units="N")
        self.add_input("altitude_econ", val=np.full(number_of_points, 0.0), units="m")
        self.add_input("true_airspeed_econ", val=np.full(number_of_points, 0.0), units="m/s")
        self.add_input("data:aerodynamics:propeller:cruise_level:altitude", units="m", val=np.nan)
        self.add_input("data:aerodynamics:propeller:sea_level:speed",
                       np.full(SPEED_PTS_NB, np.nan),
                       units="m/s", )
        self.add_input("data:aerodynamics:propeller:sea_level:thrust",
                       np.full(THRUST_PTS_NB, np.nan),
                       units="N", )
        self.add_input("data:aerodynamics:propeller:sea_level:thrust_limit",
                       np.full(SPEED_PTS_NB, np.nan),
                       units="N", )
        self.add_input("data:aerodynamics:propeller:sea_level:efficiency",
                       np.full((SPEED_PTS_NB, THRUST_PTS_NB), np.nan), )
        self.add_input("data:aerodynamics:propeller:cruise_level:speed",
                       np.full(SPEED_PTS_NB, np.nan),
                       units="m/s", )
        self.add_input("data:aerodynamics:propeller:cruise_level:thrust",
                       np.full(THRUST_PTS_NB, np.nan),
                       units="N", )
        self.add_input("data:aerodynamics:propeller:cruise_level:thrust_limit",
                       np.full(SPEED_PTS_NB, np.nan),
                       units="N", )
        self.add_input("data:aerodynamics:propeller:cruise_level:efficiency",
                       np.full((SPEED_PTS_NB, THRUST_PTS_NB), np.nan), )
        self.add_input("data:aerodynamics:propeller:installation_effect:effective_advance_ratio",
                       val=1.0, )
        self.add_input("data:aerodynamics:propeller:installation_effect:effective_efficiency:low_speed",
                       val=1.0, )
        self.add_input("data:aerodynamics:propeller:installation_effect:effective_efficiency:cruise",
                       val=1.0, )

        self.add_output(
            "mechanical_power",
            val=np.full(number_of_points, 100),
            desc="mechanical power to be supplied by all motors",
        )

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        """
        Compute the propeller efficiency.

        :param thrust: Thrust (in N)
        :param atmosphere: Atmosphere instance at intended altitude
        :return: efficiency
        """
        # Include advance ratio loss in here, we will assume that since we work at constant RPM
        # the change in advance ration is equal to a change in
        thrust = inputs["thrust_econ"]
        altitude = inputs["altitude_econ"]
        true_airspeed = inputs["true_airspeed_econ"]
        cruise_altitude_propeller = inputs["data:aerodynamics:propeller:cruise_level:altitude"]
        speed_SL = inputs["data:aerodynamics:propeller:sea_level:speed"]
        thrust_SL = inputs["data:aerodynamics:propeller:sea_level:thrust"]
        thrust_limit_SL = inputs["data:aerodynamics:propeller:sea_level:thrust_limit"]
        efficiency_SL = inputs["data:aerodynamics:propeller:sea_level:efficiency"]
        speed_CL = inputs["data:aerodynamics:propeller:cruise_level:speed"]
        thrust_CL = inputs["data:aerodynamics:propeller:cruise_level:thrust"]
        thrust_limit_CL = inputs["data:aerodynamics:propeller:cruise_level:thrust_limit"]
        efficiency_CL = inputs["data:aerodynamics:propeller:cruise_level:efficiency"]
        effective_J = inputs["data:aerodynamics:propeller:installation_effect:effective_advance_ratio"]
        effective_efficiency_ls = inputs["data:aerodynamics:propeller:installation_effect:effective_efficiency"
                                         ":low_speed"]
        effective_efficiency_cruise = inputs["data:aerodynamics:propeller:installation_effect:effective_efficiency"
                                             ":cruise"]

        installed_airspeed = true_airspeed * effective_J

        propeller_efficiency_SL = interp2d(
            thrust_SL,
            speed_SL,
            efficiency_SL * effective_efficiency_ls,  # Include the efficiency loss
            # in here
            kind="cubic",
        )
        propeller_efficiency_CL = interp2d(
            thrust_CL,
            speed_CL,
            efficiency_CL * effective_efficiency_cruise,  # Include the efficiency loss
            # in here
            kind="cubic",
        )
        if isinstance(true_airspeed, float):
            thrust_interp_SL = np.minimum(
                np.maximum(np.min(thrust_SL), thrust),
                np.interp(installed_airspeed, speed_SL, thrust_limit_SL),
            )
            thrust_interp_CL = np.minimum(
                np.maximum(np.min(thrust_CL), thrust),
                np.interp(installed_airspeed, speed_CL, thrust_limit_CL),
            )
        else:
            thrust_interp_SL = np.minimum(
                np.maximum(np.min(thrust_SL), thrust),
                np.interp(list(installed_airspeed), speed_SL, thrust_limit_SL),
            )
            thrust_interp_CL = np.minimum(
                np.maximum(np.min(thrust_CL), thrust),
                np.interp(list(installed_airspeed), speed_CL, thrust_limit_CL),
            )

        lower_bound = propeller_efficiency_SL(thrust_interp_SL, installed_airspeed)
        upper_bound = propeller_efficiency_CL(thrust_interp_CL, installed_airspeed)
        propeller_efficiency = np.interp(
            altitude, [0, cruise_altitude_propeller], [lower_bound, upper_bound]
        )

        """
        Computation of maximum thrust either due to propeller thrust limit or ICE max power.

        :param engine_setting: Engine settings (climb, cruise,... )
        :param atmosphere: Atmosphere instance at intended altitude (should be <=20km)
        :return: maximum thrust (in N)
        """

        # Calculate, for array, mechanical power @ given altitude and speed
        mechanical_power = (
                thrust * true_airspeed / propeller_efficiency
        )
        # mechanical_power = [1000 for i in range(250)]   ## TODO: comment when running full oad process
        outputs["mechanical_power"] = mechanical_power
