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

from ..constants import SUBMODEL_DEP_LANDING


@oad.RegisterSubmodel(SUBMODEL_DEP_LANDING, "fastga.submodel.performances.dep_effects_landing")
class DEPLanding(om.ExplicitComponent):
    """
    The file computes the maximum delta_Cl available to the aircraft at landing conditions and uses this
    to resize the wing to showcase the effects of DEP on the aircraft.
    """

    def setup(self):

        self.add_input("data:weight:aircraft:MTOW", val=np.nan, units="kg")

        self.add_input("data:geometry:propeller:diameter", val=np.nan, units="m")
        self.add_input("data:geometry:wing:span", val=np.nan, units="m")
        self.add_input("data:geometry:wing:area", val=10, units="m**2")
        self.add_input("data:geometry:wing:aspect_ratio", val=np.nan)
        self.add_input("data:geometry:propulsion:engine:count", val=np.nan)
        self.add_input("data:geometry:propulsion:engine:layout", val=np.nan)
        self.add_input("data:mission:sizing:takeoff:power", val=np.nan, units="W")

        self.add_input("data:aerodynamics:flaps:landing:CL_max", val=np.nan)
        self.add_input("data:aerodynamics:wing:low_speed:CL_max_clean", val=np.nan)
        self.add_input("data:aerodynamics:wing:cruise:induced_drag_coefficient", val=np.nan)
        self.add_input("data:aerodynamics:wing:cruise:CD0", val=np.nan)
        self.add_input("data:aerodynamics:flaps:landing:CD", val=np.nan)
        self.add_input("data:aerodynamics:landing_gear:low_speed:CD0", val=np.nan)
        self.add_input("data:TLAR:v_approach", val=np.nan, units="m/s")

        self.add_output("data:aerodynamics:DEP:landing:delta_Cl")
        self.add_output("data:aerodynamics:DEP:landing:delta_Cd")
        self.add_output("data:aerodynamics:DEP:landing:t_c")
        self.add_output("data:aerodynamics:DEP:landing:drag", units="N")
        self.add_output("data:aerodynamics:DEP:landing:thrust", units="N")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        MTOW = inputs["data:weight:aircraft:MTOW"]
        wing_area = inputs["data:geometry:wing:area"]
        wing_span = inputs["data:geometry:wing:span"]
        aspect_ratio = inputs["data:geometry:wing:aspect_ratio"]
        N = inputs["data:geometry:propulsion:engine:count"]
        diameter = inputs["data:geometry:propeller:diameter"]
        coeff_k = inputs["data:aerodynamics:wing:cruise:induced_drag_coefficient"]
        velocity_landing = inputs["data:TLAR:v_approach"]
        cl_max_clean = inputs["data:aerodynamics:wing:low_speed:CL_max_clean"]
        cl_max_flaps = inputs["data:aerodynamics:flaps:landing:CL_max"]
        power = inputs["data:mission:sizing:takeoff:power"]
        cd0_clean = inputs["data:aerodynamics:wing:cruise:CD0"]
        cd0_flaps = inputs["data:aerodynamics:flaps:landing:CD"]
        cd0_landing_gear = inputs["data:aerodynamics:landing_gear:low_speed:CD0"]

        cl_max_landing = cl_max_clean + cl_max_flaps
        density_landing = 1.225

        speed_of_sound = 340  # speed of sound at sea level
        mach = velocity_landing / speed_of_sound

        propeller_efficiency = 0.65  # np.mean(efficiency_SL)
        thrust_landing = power * propeller_efficiency / velocity_landing

        delta_y = 0.01
        engine_spacing = delta_y * diameter  # assumption / can also be used as user input
        dep_to_span_ratio = ((N / 2 * diameter + (N / 2 - 1) * engine_spacing) / (wing_span / 2))
        dep_to_thrust_ratio = 1  # since all the thrust comes from DEP
        propeller_distance_ratio = 0.2  # assuming the propeller is 0.2c ahead of wing (xp/c)
        propeller_wing_angle = 0  # assuming engine is parallel to wing -> alpha_w = alpha_p
        sideslip_correction_factor = 1  # assumption
        skin_friction_coefficient = 0.009

        t_c = thrust_landing / (density_landing * velocity_landing ** 2 *
                                diameter ** 2)

        a_p = 0.5 * (np.sqrt(1 + (8 * t_c) / np.pi) - 1)

        rp_c = 0.5 * np.sqrt(diameter ** 2 * aspect_ratio / aspect_ratio)
        xp_rp = propeller_distance_ratio / rp_c

        rw_rp = np.sqrt((1 + a_p) / (1 + a_p * (1 + xp_rp / np.sqrt(xp_rp ** 2 + 1))))

        a_w = ((a_p + 1) / rw_rp ** 2) - 1

        alpha_w = ((cl_max_landing / (2 * np.pi * aspect_ratio))
                   * (2 + np.sqrt(aspect_ratio ** 2 * (1 - mach ** 2) + 4)))

        delta_Cl = dep_to_span_ratio * 2 * np.pi * ((np.sin(alpha_w) - a_w * sideslip_correction_factor
                                                     * np.sin(propeller_wing_angle - alpha_w))
                                                    * np.sqrt((a_w * sideslip_correction_factor) ** 2 + 2 * a_w *
                                                              sideslip_correction_factor * np.cos(propeller_wing_angle) + 1)
                                                    - np.sin(alpha_w))

        delta_cd0 = dep_to_span_ratio * a_w ** 2 * skin_friction_coefficient

        delta_cdi = (delta_Cl ** 2 + 2 * cl_max_landing * delta_Cl) * coeff_k

        delta_Cd = delta_cd0 + delta_cdi

        cd0 = cd0_clean + cd0_flaps + cd0_landing_gear + delta_cd0
        cd = cd0 + coeff_k * cl_max_landing ** 2 + delta_cdi
        drag = cd * 0.5 * density_landing * velocity_landing ** 2 * wing_area

        outputs["data:aerodynamics:DEP:landing:delta_Cl"] = delta_Cl
        outputs["data:aerodynamics:DEP:landing:delta_Cd"] = delta_Cd
        outputs["data:aerodynamics:DEP:landing:t_c"] = t_c
        outputs["data:aerodynamics:DEP:landing:thrust"] = thrust_landing
        outputs["data:aerodynamics:DEP:landing:drag"] = drag

