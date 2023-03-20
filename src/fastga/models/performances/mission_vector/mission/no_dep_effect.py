"""FAST - Copyright (c) 2022 ONERA ISAE."""

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
import fastoad.api as oad

from ..constants import SUBMODEL_DEP_EFFECT
from stdatm import Atmosphere

oad.RegisterSubmodel.active_models[
    SUBMODEL_DEP_EFFECT
] = "fastga.submodel.performances.dep_effect.none"


@oad.RegisterSubmodel(SUBMODEL_DEP_EFFECT, "fastga.submodel.performances.dep_effect.none")
class NoDEPEffect(om.ExplicitComponent):
    """
    Using Submodels on not connected inputs will cause the code to crash, this is why we
    created this components that returns the deltas at 0 and have the same inputs as a workaround
    for that problem.
    """

    def initialize(self):
        self.options.declare(
            "number_of_points", default=1, desc="number of equilibrium to be treated"
        )
        self.options.declare(
            "flaps_position",
            default="cruise",
            desc="position of the flaps for the computation of the equilibrium",
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_input("data:weight:aircraft:MTOW", val=np.nan, units="kg")

        self.add_input("data:geometry:propeller:diameter", val=np.nan, units="m")
        self.add_input("data:geometry:wing:span", val=np.nan, units="m")
        self.add_input("data:geometry:wing:area", val=49, units="m**2")
        self.add_input("data:geometry:wing:aspect_ratio", val=np.nan)
        self.add_input(
            "data:aerodynamics:wing:low_speed:Y_vector",
            val=np.nan,
            shape_by_conn=True,
            units="m",
        )
        self.add_input(
            "data:aerodynamics:wing:low_speed:chord_vector",
            val=np.nan,
            shape_by_conn=True,
            copy_shape="data:aerodynamics:wing:low_speed:Y_vector",
            units="m",
        )
        self.add_input(
            "data:aerodynamics:wing:low_speed:CL_vector",
            val=np.nan,
            shape_by_conn=True,
            copy_shape="data:aerodynamics:wing:low_speed:Y_vector",
        )
        self.add_input("data:geometry:propulsion:engine:y_ratio", shape_by_conn=True, val=np.nan)
        self.add_input(
            "data:geometry:propulsion:nacelle:from_LE",
            shape_by_conn=True,
            copy_shape="data:geometry:propulsion:engine:y_ratio",
            val=np.nan,
            units="m",
        )
        self.add_input("data:geometry:propulsion:engine:count", val=np.nan)
        self.add_input("data:geometry:propulsion:engine:layout", val=np.nan)

        self.add_input("data:aerodynamics:wing:cruise:CL0_clean", val=np.nan)
        self.add_input("data:aerodynamics:wing:cruise:CL_alpha", val=np.nan, units="rad**-1")
        self.add_input("data:aerodynamics:wing:cruise:CM0_clean", val=np.nan)
        self.add_input("data:aerodynamics:wing:cruise:CD0", val=np.nan)
        self.add_input("data:aerodynamics:wing:cruise:induced_drag_coefficient", val=np.nan)
        self.add_input("data:TLAR:v_approach", val=np.nan, units="m/s")
        self.add_input("data:mission:sizing:takeoff:power", val=np.nan, units="W")
        self.add_input("data:aerodynamics:DEP:landing:t_c", val=np.nan)

        self.add_input("altitude", val=np.full(number_of_points, np.nan), units="m")
        self.add_input("true_airspeed", val=np.full(number_of_points, np.nan), units="m/s")

        self.add_input("alpha", val=np.full(number_of_points, np.nan), units="rad")
        self.add_input("thrust", val=np.full(number_of_points, 0.0), units="N")

        self.add_output("delta_Cl", val=np.full(number_of_points, 0.0))
        self.add_output("delta_Cd", val=np.full(number_of_points, 0.0))
        self.add_output("delta_Cm", val=np.full(number_of_points, 0.0))
        self.add_output("data:aerodynamics:DEP:max_delta_Cl", shape=1)
        self.add_output("data:aerodynamics:DEP:max_delta_Cd", shape=1)
        self.add_output("data:aerodynamics:DEP:max_flight_point", shape=1)
        self.add_output("data:aerodynamics:DEP:DEP_to_span")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        MTOW = inputs["data:weight:aircraft:MTOW"]
        thrust = np.clip(inputs["thrust"], 0, None)
        velocity = inputs["true_airspeed"]
        wing_area = inputs["data:geometry:wing:area"]
        wing_span = inputs["data:geometry:wing:span"]
        aspect_ratio = inputs["data:geometry:wing:aspect_ratio"]
        N = inputs["data:geometry:propulsion:engine:count"]
        diameter = inputs["data:geometry:propeller:diameter"]
        alpha = np.clip(inputs["alpha"], 0, None)
        cl_0 = inputs["data:aerodynamics:wing:cruise:CL0_clean"]
        cl_alpha = inputs["data:aerodynamics:wing:cruise:CL_alpha"]
        coeff_k = inputs["data:aerodynamics:wing:cruise:induced_drag_coefficient"]
        velocity_landing = inputs["data:TLAR:v_approach"]
        power = inputs["data:mission:sizing:takeoff:power"]

        altitude = inputs["altitude"]
        density = Atmosphere(altitude, altitude_in_feet=False).density
        density_landing = 1.225
        speed_of_sound = Atmosphere(altitude, altitude_in_feet=False).speed_of_sound
        mach = velocity / speed_of_sound

        cl = cl_0 + cl_alpha * alpha

        delta_y = 0.01
        engine_spacing = delta_y * diameter  # assumption / can also be used as user input
        dep_to_span_ratio = ((N/2 * diameter + (N/2 - 1) * engine_spacing) / (wing_span/2))
        dep_to_thrust_ratio = 1  # since all the thrust comes from DEP
        propeller_distance_ratio = 0.2  # assuming the propeller is 0.2c ahead of wing (xp/c)
        propeller_distance_ratio = 0.2  # assuming the propeller is 0.25c ahead of wing
        propeller_wing_angle = 0  # assuming engine is parallel to wing -> alpha_w = alpha_p
        sideslip_correction_factor = 1  # assumption
        skin_friction_coefficient = 0.009

        propeller_efficiency = 0.65 #np.mean(efficiency_SL)
        thrust_landing = power * propeller_efficiency / velocity_landing

        t_c_landing = inputs["data:aerodynamics:DEP:landing:t_c"]
        t_c = t_c_landing * ((thrust * density_landing * velocity_landing ** 2) /
                             (thrust_landing * density * velocity ** 2))

        a_p = 0.5 * (np.sqrt(1 + (8 * t_c) / np.pi) - 1)

        rp_c = 0.5 * np.sqrt(diameter ** 2 * aspect_ratio / aspect_ratio)
        xp_rp = propeller_distance_ratio / rp_c

        rw_rp = np.sqrt((1 + a_p) / (1 + a_p * (1 + xp_rp / np.sqrt(xp_rp ** 2 + 1))))

        a_w = ((a_p + 1) / rw_rp ** 2) - 1

        alpha_w = ((cl / (2 * np.pi * aspect_ratio))
                   * (2 + np.sqrt(aspect_ratio ** 2 * (1 - mach ** 2) + 4)))

        delta_Cl = dep_to_span_ratio * 2 * np.pi * ((np.sin(alpha_w) - a_w * sideslip_correction_factor
                                                     * np.sin(propeller_wing_angle - alpha_w))
                                                    * np.sqrt((a_w * sideslip_correction_factor) ** 2 + 2 * a_w *
                                                              sideslip_correction_factor * np.cos(propeller_wing_angle) + 1)
                                                    - np.sin(alpha_w))

        delta_cd0 = dep_to_span_ratio * a_w ** 2 * skin_friction_coefficient

        delta_cdi = (delta_Cl ** 2 + 2 * cl * delta_Cl) * coeff_k

        delta_Cd = delta_cd0 + delta_cdi

        max_delta_Cl = np.amax(delta_Cl[1:251])
        index = np.where(delta_Cl == max_delta_Cl)[0][0]
        max_delta_Cd = delta_Cd[index]

        outputs["data:aerodynamics:DEP:DEP_to_span"] = dep_to_span_ratio
        outputs["data:aerodynamics:DEP:max_delta_Cl"] = max_delta_Cl
        outputs["data:aerodynamics:DEP:max_delta_Cd"] = max_delta_Cd
        outputs["data:aerodynamics:DEP:max_flight_point"] = index
        outputs["delta_Cl"] = delta_Cl
        outputs["delta_Cd"] = delta_Cd
        outputs["delta_Cm"] = 0
