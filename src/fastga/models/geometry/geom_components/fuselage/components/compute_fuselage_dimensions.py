"""
    Estimation of geometry of fuselage part A - Cabin (Commercial).
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

import numpy as np
import math
from openmdao.core.explicitcomponent import ExplicitComponent


class ComputeFuselageGeometryBasic(ExplicitComponent):
    """
    Geometry of fuselage - Cabin length defined with total fuselage length input (no sizing).
    """

    def setup(self):

        self.add_input("data:geometry:fuselage:length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:front_length", val=np.nan, units="m")
        self.add_input("data:geometry:fuselage:rear_length", val=np.nan, units="m")

        self.add_output("data:geometry:cabin:length", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        fus_length = inputs["data:geometry:fuselage:length"]
        lav = inputs["data:geometry:fuselage:front_length"]
        lar = inputs["data:geometry:fuselage:rear_length"]

        # Cabin total length
        cabin_length = fus_length - (lav + lar)

        outputs["data:geometry:cabin:length"] = cabin_length


class ComputeFuselageGeometryCabinSizingFD(ExplicitComponent):
    # TODO: Document equations. Cite sources
    """
    Geometry of fuselage - Cabin is sized based on layout (seats, aisle...) and HTP/VTP position
    (Fixed tail Distance).
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._engine_wrapper = None

    def initialize(self):
        self.options.declare("propulsion_id", default="", types=str)

    def setup(self):

        self.add_input("data:geometry:cabin:seats:passenger:NPAX_max", val=np.nan)
        self.add_input("data:geometry:cabin:seats:pilot:length", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:pilot:width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:length", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:count_by_row", val=np.nan)
        self.add_input("data:geometry:cabin:aisle_width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:luggage:mass_max", val=np.nan, units="kg")
        self.add_input("data:geometry:propeller:depth", val=np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:at25percent:x", val=np.nan, units="m")
        self.add_input(
            "data:geometry:horizontal_tail:MAC:at25percent:x:from_wingMAC25", val=np.nan, units="m"
        )
        self.add_input(
            "data:geometry:vertical_tail:MAC:at25percent:x:from_wingMAC25", val=np.nan, units="m"
        )
        self.add_input("data:geometry:horizontal_tail:MAC:length", val=np.nan, units="m")
        self.add_input("data:geometry:vertical_tail:MAC:length", val=np.nan, units="m")
        self.add_input("data:geometry:horizontal_tail:sweep_25", val=np.nan, units="deg")
        self.add_input("data:geometry:horizontal_tail:span", val=np.nan, units="m")
        self.add_input("data:geometry:vertical_tail:sweep_25", val=np.nan, units="deg")
        self.add_input("data:geometry:vertical_tail:span", val=np.nan, units="m")
        self.add_input("data:geometry:propulsion:nacelle:length", val=np.nan, units="m")
        self.add_input("data:geometry:propulsion:engine:layout", val=np.nan)

        self.add_output("data:geometry:cabin:NPAX")
        self.add_output("data:geometry:aircraft:length", units="m")
        self.add_output("data:geometry:fuselage:length", val=10.0, units="m")
        self.add_output("data:geometry:fuselage:maximum_width", units="m")
        self.add_output("data:geometry:fuselage:maximum_height", units="m")
        self.add_output("data:geometry:fuselage:front_length", units="m")
        self.add_output("data:geometry:fuselage:rear_length", units="m")
        self.add_output("data:geometry:fuselage:PAX_length", units="m")
        self.add_output("data:geometry:cabin:length", units="m")
        self.add_output("data:geometry:fuselage:luggage_length", units="m")

        self.declare_partials(
            "*", "*", method="fd"
        )  # FIXME: declare proper partials without int values

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        nacelle_length = inputs["data:geometry:propulsion:nacelle:length"]
        npax_max = inputs["data:geometry:cabin:seats:passenger:NPAX_max"]
        l_pilot_seats = inputs["data:geometry:cabin:seats:pilot:length"]
        w_pilot_seats = inputs["data:geometry:cabin:seats:pilot:width"]
        l_pass_seats = inputs["data:geometry:cabin:seats:passenger:length"]
        w_pass_seats = inputs["data:geometry:cabin:seats:passenger:width"]
        seats_p_row = inputs["data:geometry:cabin:seats:passenger:count_by_row"]
        w_aisle = inputs["data:geometry:cabin:aisle_width"]
        luggage_mass_max = inputs["data:geometry:cabin:luggage:mass_max"]
        prop_layout = inputs["data:geometry:propulsion:engine:layout"]
        spinner_length = inputs["data:geometry:propeller:depth"]
        fa_length = inputs["data:geometry:wing:MAC:at25percent:x"]
        ht_lp = inputs["data:geometry:horizontal_tail:MAC:at25percent:x:from_wingMAC25"]
        vt_lp = inputs["data:geometry:vertical_tail:MAC:at25percent:x:from_wingMAC25"]
        ht_length = inputs["data:geometry:horizontal_tail:MAC:length"]
        vt_length = inputs["data:geometry:vertical_tail:MAC:length"]
        sweep_25_vt = inputs["data:geometry:vertical_tail:sweep_25"]
        b_v = inputs["data:geometry:vertical_tail:span"]
        sweep_25_ht = inputs["data:geometry:horizontal_tail:sweep_25"]
        b_h = inputs["data:geometry:horizontal_tail:span"]

        # Length of instrument panel
        l_instr = 0.7
        # Length of pax cabin
        npax = math.ceil(float(npax_max) / float(seats_p_row)) * float(seats_p_row)
        n_rows = npax / float(seats_p_row)
        l_pax = l_pilot_seats + n_rows * l_pass_seats
        # Cabin width considered is for side by side seats
        w_cabin = max(2 * w_pilot_seats, seats_p_row * w_pass_seats + w_aisle)
        r_i = w_cabin / 2
        radius = 1.06 * r_i
        # Cylindrical fuselage
        b_f = 2 * radius
        # 0.14m is the distance between both lobe centers of the fuselage
        h_f = b_f + 0.14
        # Luggage length (80% of internal radius section can be filled with luggage)
        luggage_density = 161.0  # In kg/m3
        l_lug = (luggage_mass_max / luggage_density) / (0.8 * math.pi * r_i ** 2)
        # Cabin total length
        cabin_length = l_instr + l_pax + l_lug
        # Calculate nose length
        if prop_layout == 3.0:  # engine located in nose
            lav = nacelle_length + spinner_length
        else:
            lav = 1.40 * h_f
            # Used to be 1.7, supposedly as an A320 according to FAST legacy. Results on the BE76
            # tend to say it is around 1.40, though it varies a lot depending on the airplane and
            # its use
        # Calculate fuselage length
        fus_length = fa_length + max(ht_lp + 0.75 * ht_length, vt_lp + 0.75 * vt_length)
        plane_length = fa_length + max(
            ht_lp + 0.75 * ht_length + b_h / 2.0 * math.tan(sweep_25_ht * math.pi / 180),
            vt_lp + 0.75 * vt_length + b_v * math.tan(sweep_25_vt * math.pi / 180),
        )
        lar = fus_length - (lav + cabin_length)

        outputs["data:geometry:cabin:NPAX"] = npax
        outputs["data:geometry:fuselage:length"] = fus_length
        outputs["data:geometry:aircraft:length"] = plane_length
        outputs["data:geometry:fuselage:maximum_width"] = b_f
        outputs["data:geometry:fuselage:maximum_height"] = h_f
        outputs["data:geometry:fuselage:front_length"] = lav
        outputs["data:geometry:fuselage:rear_length"] = lar
        outputs["data:geometry:fuselage:PAX_length"] = l_pax
        outputs["data:geometry:cabin:length"] = cabin_length
        outputs["data:geometry:fuselage:luggage_length"] = l_lug


class ComputeFuselageGeometryCabinSizingFL(ExplicitComponent):
    # TODO: Document equations. Cite sources
    """
    Geometry of fuselage - Cabin is sized based on layout (seats, aisle...) and additional rear
    length (Fixed Length).
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._engine_wrapper = None

    def initialize(self):
        self.options.declare("propulsion_id", default="", types=str)

    def setup(self):

        self.add_input("data:geometry:cabin:seats:passenger:NPAX_max", val=np.nan)
        self.add_input("data:geometry:cabin:seats:pilot:length", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:pilot:width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:length", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:count_by_row", val=np.nan)
        self.add_input("data:geometry:cabin:aisle_width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:luggage:mass_max", val=np.nan, units="kg")
        self.add_input("data:geometry:fuselage:rear_length", units="m")
        self.add_input("data:geometry:propulsion:nacelle:length", val=np.nan, units="m")
        self.add_input("data:geometry:propulsion:engine:layout", val=np.nan)

        self.add_output("data:geometry:cabin:NPAX")
        self.add_output("data:geometry:fuselage:length", val=10.0, units="m")
        self.add_output("data:geometry:fuselage:maximum_width", units="m")
        self.add_output("data:geometry:fuselage:maximum_height", units="m")
        self.add_output("data:geometry:fuselage:front_length", units="m")
        self.add_output("data:geometry:fuselage:PAX_length", units="m")
        self.add_output("data:geometry:cabin:length", units="m")
        self.add_output("data:geometry:fuselage:luggage_length", units="m")

        self.declare_partials(
            "*", "*", method="fd"
        )  # FIXME: declare proper partials without int values

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        nacelle_length = inputs["data:geometry:propulsion:nacelle:length"]
        npax_max = inputs["data:geometry:cabin:seats:passenger:NPAX_max"]
        l_pilot_seats = inputs["data:geometry:cabin:seats:pilot:length"]
        w_pilot_seats = inputs["data:geometry:cabin:seats:pilot:width"]
        l_pass_seats = inputs["data:geometry:cabin:seats:passenger:length"]
        w_pass_seats = inputs["data:geometry:cabin:seats:passenger:width"]
        seats_p_row = inputs["data:geometry:cabin:seats:passenger:count_by_row"]
        w_aisle = inputs["data:geometry:cabin:aisle_width"]
        luggage_mass_max = inputs["data:geometry:cabin:luggage:mass_max"]
        prop_layout = inputs["data:geometry:propulsion:engine:layout"]
        lar = inputs["data:geometry:fuselage:rear_length"]

        # Length of instrument panel
        l_instr = 0.7
        # Length of pax cabin
        # noinspection PyBroadException
        npax = math.ceil(float(npax_max) / float(seats_p_row)) * float(seats_p_row)
        n_rows = npax / float(seats_p_row)
        l_pax = l_pilot_seats + n_rows * l_pass_seats
        # Cabin width considered is for side by side seats
        w_cabin = max(2 * w_pilot_seats, seats_p_row * w_pass_seats + w_aisle)
        r_i = w_cabin / 2
        radius = 1.06 * r_i
        # Cylindrical fuselage
        b_f = 2 * radius
        # 0.14m is the distance between both lobe centers of the fuselage
        h_f = b_f + 0.14
        # Luggage length (80% of internal radius section can be filled with luggage)
        luggage_density = 161.0  # In kg/m3
        l_lug = (luggage_mass_max / luggage_density) / (0.8 * math.pi * r_i ** 2)
        # Cabin total length
        cabin_length = l_instr + l_pax + l_lug
        # Calculate nose length
        if prop_layout == 3.0:  # engine located in nose
            lav = nacelle_length
        else:
            lav = 1.7 * h_f
            # Calculate fuselage length
        fus_length = lav + cabin_length + lar

        outputs["data:geometry:cabin:NPAX"] = npax
        outputs["data:geometry:fuselage:length"] = fus_length
        outputs["data:geometry:fuselage:maximum_width"] = b_f
        outputs["data:geometry:fuselage:maximum_height"] = h_f
        outputs["data:geometry:fuselage:front_length"] = lav
        outputs["data:geometry:fuselage:PAX_length"] = l_pax
        outputs["data:geometry:cabin:length"] = cabin_length
        outputs["data:geometry:fuselage:luggage_length"] = l_lug


class ComputeFuselageGeometryCabinSizingH2(ExplicitComponent):
    # TODO: Document equations. Cite sources
    """
    Geometry of fuselage - Cabin is sized based on layout (seats, aisle...) and additional rear
    length (Fixed Length). Additional fuselage size for hydrogen storage system and fuel cell
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._engine_wrapper = None

    def initialize(self):
        self.options.declare("propulsion_id", default="", types=str)

    def setup(self):

        self.add_input("data:geometry:cabin:seats:passenger:NPAX_max", val=np.nan)
        self.add_input("data:geometry:cabin:seats:pilot:length", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:pilot:width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:length", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:seats:passenger:count_by_row", val=np.nan)
        self.add_input("data:geometry:cabin:aisle_width", val=np.nan, units="m")
        self.add_input("data:geometry:cabin:luggage:mass_max", val=np.nan, units="kg")
        self.add_input("data:geometry:fuselage:rear_length", units="m")
        self.add_input("data:geometry:propulsion:nacelle:length", val=np.nan, units="m")
        self.add_input("data:geometry:propulsion:engine:layout", val=np.nan)
        self.add_input("data:geometry:propulsion:hydrogen:weight", val=np.nan, units="kg") #additional code
        self.add_input("data:geometry:propulsion:hydrogen:volume_liquid", val=np.nan, units="m**3") #additional code
        self.add_input("data:geometry:fuselage:number_of_tank", val=np.nan) #additional code
        self.add_input("data:geometry:propulsion:fuelcell:volume", val=np.nan) #additional code (1)
        self.add_input("data:geometry:propulsion:fuelcell:stacks", val=np.nan) #additional code

        self.add_output("data:geometry:cabin:NPAX")
        self.add_output("data:geometry:fuselage:length", val=10.0, units="m")
        self.add_output("data:geometry:fuselage:maximum_width", units="m")
        self.add_output("data:geometry:fuselage:maximum_height", units="m")
        self.add_output("data:geometry:fuselage:front_length", units="m")
        self.add_output("data:geometry:fuselage:PAX_length", units="m")
        self.add_output("data:geometry:cabin:length", units="m")
        self.add_output("data:geometry:fuselage:luggage_length", units="m")
        self.add_output("data:geometry:fuselage:tank_length", units="m") #additional code
        self.add_output("data:geometry:fuselage:gravimetric_index") #additional code
        self.add_output("data:geometry:fuselage:a", units="m") #additional code
        self.add_output("data:geometry:fuselage:b", units="m") #additional code
        self.add_output("data:geometry:fuselage:c", units="m") #additional code
        self.add_output("data:geometry:fuselage:H2_storage_mass", units="kg") #additional code
        self.add_output("data:geometry:fuselage:radius", units="m") #additional code
        self.add_output("data:geometry:fuselage:fuelcell:length", units="m") #additional code
        self.add.output("data:geometry:fuselage:tank_thickness", units="m") #additional code
        self.add.output("data:geometry:fuselage:shell_thickness", units="m") #additional code

        self.declare_partials(
            "*", "*", method="fd"
        )  # FIXME: declare proper partials without int values

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        nacelle_length = inputs["data:geometry:propulsion:nacelle:length"]
        npax_max = inputs["data:geometry:cabin:seats:passenger:NPAX_max"]
        l_pilot_seats = inputs["data:geometry:cabin:seats:pilot:length"]
        w_pilot_seats = inputs["data:geometry:cabin:seats:pilot:width"]
        l_pass_seats = inputs["data:geometry:cabin:seats:passenger:length"]
        w_pass_seats = inputs["data:geometry:cabin:seats:passenger:width"]
        seats_p_row = inputs["data:geometry:cabin:seats:passenger:count_by_row"]
        w_aisle = inputs["data:geometry:cabin:aisle_width"]
        luggage_mass_max = inputs["data:geometry:cabin:luggage:mass_max"]
        prop_layout = inputs["data:geometry:propulsion:engine:layout"]
        lar = inputs["data:geometry:fuselage:rear_length"]
        H2_mass = inputs["data:geometry:propulsion:hydrogen:weight"] #additional code
        H2_volume = inputs["data:geometry:propulsion:hydrogen:volume_liquid"] #additional code
        ntank = inputs["data:geometry:fuselage:number_of_tank"] #additional code
        fuelcell_volume = inputs["data:geometry:propulsion:fuelcell:volume"] #additional code
        fuelcell_number = inputs["data:geometry:propulsion:fuelcell:stacks"] #additional code

        # Length of instrument panel
        l_instr = 0.7
        # Length of pax cabin
        # noinspection PyBroadException
        npax = math.ceil(float(npax_max) / float(seats_p_row)) * float(seats_p_row)
        n_rows = npax / float(seats_p_row)
        l_pax = l_pilot_seats + n_rows * l_pass_seats
        # Cabin width considered is for side by side seats
        w_cabin = max(2 * w_pilot_seats, seats_p_row * w_pass_seats + w_aisle)
        r_i = w_cabin / 2
        radius = 1.06 * r_i
        # Cylindrical fuselage
        b_f = 2 * radius
        # 0.14m is the distance between both lobe centers of the fuselage
        h_f = b_f + 0.14
        # Luggage length (80% of internal radius section can be filled with luggage)
        luggage_density = 161.0  # In kg/m3
        l_lug = (luggage_mass_max / luggage_density) / (0.8 * math.pi * r_i ** 2)

        # Hydrogen tank sizing upgraded model, additional code

        # Constant
        LH2_density = 71  # unit: kg/m^3
        shell_density = 2480  # 830.6 #unit: kg/m^3 for Kevlar epoxy composite material
        ins_density = 35  # unit: kg/m^3 for foam material
        p_vent = 3 * 10 ** 5  # unit: Pa, venting pressure
        p_out = 0.752 * 10 ** 5  # unit: Pa, air pressure outside the tank
        e_w = 0.8  # safety factor
        sigma_aR1 = 172  # unit: MPa
        sigma_b = 234  # unit: MPa
        R_1 = 0.43
        R_2 = 0.43
        h_fg = 446592  # unit: J/kg, latent heat of vaporisation of LH2
        T_s = 293  # unit: K, outside surface temperature of the insulation
        T_LH2 = 20  # unit: K, cryogenic temperature of LH2
        k_ins = 0.0064  # unit: W/(m.K), thermal conductivity of insulation material
        psi = 0.1  # ratio of b/c
        N_e = 2  # number of engine

        # Model of tank inner dimension
        H2_volume_corr = (100 / 97) * H2_volume  # inner volume including venting, contraction-expansion, ullage, internal equipment
        a = 0.8 * r_i
        c = 0.8 * r_i
        b = psi * c
        ls = H2_volume_corr / (math.pi * a * c) - 4 / 3 * b
        l_tank = ls + 2 * b

        # Model of tank structural shell/wall
        sigma_a = sigma_aR1 / (1 - (sigma_aR1 * 0.5 * (1 + R_1)) / sigma_b)
        sigma = sigma_a / (1 + (sigma_a * 0.5 * (1 + R_2)) / sigma_b)
        t_shell = (p_vent - p_out) * a / (sigma * 10 ** 6 * e_w)
        H2_volume_shell = (math.pi * (a + t_shell) * (c + t_shell) * ((ls + t_shell) + 4 / 3 * (b + t_shell))) - (
                    math.pi * a * c * (ls + 4 / 3 * b))
        m_shell = H2_volume_shell * shell_density

        # Model of tank insulation
        m_dot = 0.016 * H2_mass / 3600
        A_ins = 2 * math.pi * (0.8 * r_i + t_shell) * l_tank
        t_ins = k_ins * A_ins * (T_s - T_LH2) / (m_dot * h_fg)
        H2_volume_ins = (math.pi * (a + t_shell + t_ins) * (c + t_shell + t_ins) * (
                    (ls + t_shell + t_ins) + 4 / 3 * (b + t_shell + t_ins))) - (
                                    math.pi * (a + t_shell) * (c + t_shell) * ((ls + t_shell) + 4 / 3 * (b + t_shell)))
        m_ins = H2_volume_ins * ins_density

        # Model of hydrogen storage system mass
        m_system = 36.3 * (N_e + ntank - 1) + 4.366 * ntank ** 0.5 * H2_volume ** 0.333
        H2_storage_mass = H2_mass + 1.018 * (m_shell + m_ins + m_system)  # including 1.8% of tank support system
        gravimetric_efficiency = H2_mass / (H2_storage_mass - m_system)

        # Fuel cell length modelling, additional code
        fuelcell_height = 0.8 * (2 * radius / 2 ** (1 / 2))
        fuelcell_area = fuelcell_height ** 2
        fuelcell_length_each = fuelcell_volume / (fuelcell_area * fuelcell_number)  # length of the fuel cell
        l_fuelcell = fuelcell_length_each * fuelcell_number + 0.1 * fuelcell_length_each  # gap betwenn fuel cell is assumed 10% of each length

        # Cabin total length
        cabin_length = l_instr + l_pax + l_lug + l_tank + l_fuelcell #additional code
        # Calculate nose length
        if prop_layout == 3.0:  # engine located in nose
            lav = nacelle_length
        else:
            lav = 1.7 * h_f
            # Calculate fuselage length
        fus_length = lav + cabin_length + lar

        outputs["data:geometry:cabin:NPAX"] = npax
        outputs["data:geometry:fuselage:length"] = fus_length
        outputs["data:geometry:fuselage:maximum_width"] = b_f
        outputs["data:geometry:fuselage:maximum_height"] = h_f
        outputs["data:geometry:fuselage:front_length"] = lav
        outputs["data:geometry:fuselage:PAX_length"] = l_pax
        outputs["data:geometry:cabin:length"] = cabin_length
        outputs["data:geometry:fuselage:luggage_length"] = l_lug
        outputs["data:geometry:fuselage:tank_length"] = l_tank #additional code
        outputs["data:geometry:fuselage:gravimetric_index"] = gravimetric_index #additional code
        outputs["data:geometry:fuselage:a"] = a #additional code
        outputs["data:geometry:fuselage:b"] = b #additional code
        outputs["data:geometry:fuselage:c"] = c #additional code
        outputs["data:geometry:fuselage:H2_storage_mass"] = H2_storage_mass #additional code
        outputs["data:geometry:fuselage:radius"] = r_i #additional code
        outputs["data:geometry:fuselage:fuelcell:length"] = l_fuelcell #additional code
        outputs["data:geometry:fuselage:tank_thickness"] = t_ins #additional code
        outputs["data:geometry:fuselage:shell_thickness"] = t_shell #additional code