# -*- coding: utf-8 -*-
"""
Created on Tue Oct 18 15:18:28 2022

@author: Charan
"""
import logging
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

import fastoad.api as oad
import openmdao.api as om

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_FUELCELL_PARAMETERS

_LOGGER = logging.getLogger(__name__)


@oad.RegisterSubmodel(SUBMODEL_FUELCELL_PARAMETERS,
                      "fastga.submodel.propulsion.hybrid_propulsion.fuelcell.legacy")
class FuelcellParameters(om.ExplicitComponent):
    def initialize(self):
        self.options.declare(
            "number_of_points", default=252, desc="number of equilibrium to be treated"
        )

    def setup(self):
        number_of_points = self.options["number_of_points"]

        self.add_input("mechanical_power", shape=number_of_points)
        self.add_input("data:mission:sizing:takeoff:duration", units="s")
        self.add_input("data:propulsion:fuelcell:current", units="A")
        self.add_input("data:propulsion:system_voltage", units="V")

        self.add_input("data:propulsion:fuelcell:efficiency", val=0.50)
        self.add_input("data:propulsion:motor:efficiency", val=0.93)
        self.add_input("data:propulsion:switch:efficiency", val=0.97)
        self.add_input("data:propulsion:gearbox:efficiency", val=0.98)
        self.add_input("data:propulsion:controller:efficiency", val=0.97)
        self.add_input("data:propulsion:bus:efficiency", val=0.97)
        self.add_input("data:propulsion:converter:efficiency", val=0.97)
        self.add_input("data:propulsion:cables:efficiency", val=0.99)

        self.add_input("P_oper", val=0.753)
        self.add_input("P_nom", val=1.0)
        self.add_input("single_stack_mass", val=42.0)
        self.add_input("single_stack_volume", val=0.03721536)  # in m3
        self.add_input("cells_in_1stack", val=455)
        self.add_input("lambdaO2", val=1.7)
        self.add_input("H2_density_300bar", val=20.0)
        self.add_input("H2_density_700bar", val=40.0)
        self.add_input("H2_density_liq", val=70.0)
        self.add_input("H2_storageEff_300bar", val=0.05)
        self.add_input("H2_storageEff_700bar", val=0.10)
        self.add_input("H2_storageEff_liq", val=0.20)
        self.add_input("H2_volumeEff_300bar", val=0.50)
        self.add_input("H2_volumeEff_700bar", val=0.50)
        self.add_input("H2_volumeEff_liq", val=0.50)
        self.add_input("margin", val=1.20)
        # PEM FC paramters
        self.add_input("V_thermo", val=1.22)
        self.add_input("Top", val=323.15)
        self.add_input("Rgas", val=8.315)
        self.add_input("n", val=2.0)
        self.add_input("F", val=96485.0)
        self.add_input("R", val=0.0001)
        self.add_input("alpha_fuelcell", val=0.195)
        self.add_input("Io", val=0.46)
        self.add_input("I_leak", val=10)
        self.add_input("I_limit", val=380)
        self.add_input("c", val=0.17)
        self.add_input("mu_H2", val=0.95)

        self.add_input(
            "altitude_econ",
            shape=number_of_points,
            val=np.full(number_of_points, 2000),
            units="m",
        )
        self.add_input(
            "time_step_econ",
            shape=number_of_points,
            val=np.full(number_of_points, 0.1),
            units="s",
        )

        self.add_output(
            "fuelcell_weight",
            val=1925.00,
            units="kg",
            desc="mass of all fuelcells",
        )

        self.add_output("fuelcell_Pelec_max", shape=1)
        self.add_output("fuel_consumed_t_econ", val=0, shape=number_of_points)
        self.add_output("data:geometry:propulsion:fuelcell:stacks", shape=1)
        self.add_output("data:geometry:propulsion:fuelcell:weight", units="kg", shape=1)
        self.add_output("data:geometry:propulsion:fuelcell:volume", units="m**3", shape=1)
        self.add_output("data:geometry:propulsion:hydrogen:weight", units="kg", shape=1)
        self.add_output("data:geometry:propulsion:fuelcell:air_flow", shape=1)
        self.add_output("data:geometry:propulsion:hydrogen:volume_300bar", units="m**3", shape=1)
        self.add_output("data:geometry:propulsion:hydrogen:volume_700bar", units="m**3", shape=1)
        self.add_output("data:geometry:propulsion:hydrogen:volume_liquid", units="m**3", shape=1)

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        _LOGGER.debug("Calculating fuelcell parameters")
        total_efficiency = inputs["data:propulsion:motor:efficiency"] * inputs["data:propulsion:gearbox:efficiency"] \
                           * inputs["data:propulsion:controller:efficiency"] \
                           * inputs["data:propulsion:switch:efficiency"] * inputs["data:propulsion:bus:efficiency"] \
                            * inputs["data:propulsion:converter:efficiency"] \
                           * inputs["data:propulsion:cables:efficiency"]

        time_step = inputs["time_step_econ"]
        FCeff = inputs["data:propulsion:fuelcell:efficiency"]
        cell_current = inputs["data:propulsion:fuelcell:current"]
        V_max = inputs["data:propulsion:system_voltage"]
        P_oper = inputs["P_oper"]
        P_nom = inputs["P_nom"]
        single_stack_mass = inputs["single_stack_mass"]
        single_stack_volume = inputs["single_stack_volume"]
        cells_in_1stack = inputs["cells_in_1stack"]
        lambdaO2 = inputs["lambdaO2"]
        H2_density_300bar = inputs["H2_density_300bar"]
        H2_density_700bar = inputs["H2_density_700bar"]
        H2_density_liq = inputs["H2_density_liq"]
        margin = inputs["margin"]
        V_thermo = inputs["V_thermo"]
        Top = inputs["Top"]
        Rgas = inputs["Rgas"]
        n = inputs["n"]
        F = inputs["F"]
        R = inputs["R"]
        alpha_fuelcell = inputs["alpha_fuelcell"]
        Io = inputs["Io"]
        I_limit = inputs["I_limit"]
        I_leak = inputs["I_leak"]
        c = inputs["c"]
        mu_H2 = float(inputs["mu_H2"])

        mechanical_power = inputs["mechanical_power"]
        # power and time for cruise and climb
        electrical_power_cruise_max = max(mechanical_power[100:200] / total_efficiency)
        electrical_power_cruise = mechanical_power[100:200] / total_efficiency
        Cruise_Pelec_max = electrical_power_cruise_max
        Climb_Pelec = electrical_power_cruise_max
        TO_Pelec = electrical_power_cruise_max
        Cruise_Pelec = electrical_power_cruise
        Cruise_time = time_step[100:200]
        Climb_time = time_step[0:100]

        # time for TO
        TO_time = inputs["data:mission:sizing:takeoff:duration"]

        # power and time for descent
        electrical_power_descent = mechanical_power[200:250] / total_efficiency
        Descent_Pelec = electrical_power_descent
        Taxi_Pelec = 0  # Taxi is not considered but can be as <<mechanical_power[250:252] / total_efficiency>>
        Descent_time = time_step[200:250]
        Taxi_time = time_step[250:252]

        Climb_time_sum = sum(Climb_time)
        Cruise_time_sum = sum(Cruise_time)
        Descent_time_sum = sum(Descent_time)
        print("climb, cruise, descent time are", Climb_time_sum, Cruise_time_sum, Descent_time_sum)

        # fus_dia = inputs["data:geometry:fuselage:length"]
        I = list(range(0, 210, 10))  # Current [A]
        V_cell = []  # Cell voltage [V]

        # Calculating cell voltage for a range of current
        for i in range(len(I)):
            V_act = Rgas * Top * (np.log(I[i] + I_leak) - np.log(Io)) / (
                    alpha_fuelcell * n * F)
            V_ohm = I[i] * R
            V_conc = c * np.log(I_limit / (I_limit - I[i] - I_leak))
            delVp = 0.06 * np.log(P_oper / P_nom)
            V_cell.append(float(V_thermo - V_act - V_ohm - V_conc + delVp))


        # User chooses the current to determine cell voltage (user input in .xml file)
        cell_current = float(cell_current)
        f_voltage = interp1d(I, V_cell)
        cell_voltage = float(f_voltage(cell_current))

        # Take-off
        TO_Pelec_inFC = TO_Pelec / FCeff
        TO_H2_mass = (1.05 * 1e-8 * TO_Pelec_inFC / (mu_H2 * cell_voltage)) * TO_time
        TO_air_massflow = 2.856 * (1e-7) * lambdaO2 * TO_Pelec_inFC

        # Climb
        Climb_Pelec_inFC = Climb_Pelec / FCeff
        Climb_H2_mass = (1.05 * 1e-8 * Climb_Pelec_inFC / (mu_H2 * cell_voltage)) * Climb_time
        Climb_air_massflow = 2.856 * (1e-7) * lambdaO2 * Climb_Pelec_inFC

        # Cruise
        Cruise_Imax = Cruise_Pelec_max / V_max
        Cruise_stacks_parallel = np.ceil(Cruise_Imax / cell_current)
        Cruise_cells_in_series = np.ceil(V_max / cell_voltage)
        Cruise_stacks_series = np.ceil(Cruise_cells_in_series / cells_in_1stack)
        Cruise_stacks = Cruise_stacks_parallel * Cruise_stacks_series
        Cruise_stack_mass = Cruise_stacks * single_stack_mass

        Cruise_Pelec_inFC = Cruise_Pelec / FCeff
        Cruise_H2_mass = (1.05 * 1e-8 * Cruise_Pelec_inFC / (mu_H2 * cell_voltage)) * Cruise_time
        Cruise_air_massflow = 2.856 * (1e-7) * lambdaO2 * Cruise_Pelec_inFC

        # Descent
        Descent_Pelec_inFC = Descent_Pelec / FCeff
        Taxi_Pelec_inFC = Taxi_Pelec / FCeff
        Descent_H2_mass = (1.05 * 1e-8 * Descent_Pelec_inFC / (mu_H2 * cell_voltage)) * Descent_time
        Taxi_H2_mass = (1.05 * 1e-8 * Taxi_Pelec_inFC / (mu_H2 * cell_voltage)) * Taxi_time
        Descent_air_massflow = 2.856 * (1e-7) * lambdaO2 * Descent_Pelec_inFC
        Taxi_air_massflow = 2.856 * (1e-7) * lambdaO2 * Taxi_Pelec_inFC

        # FC stack mass
        FC_stack_mass = Cruise_stack_mass

        # FC stack volume
        FC_stack_volume = Cruise_stacks * single_stack_volume

        # H2 mass
        H2_mass = sum(TO_H2_mass) + sum(Climb_H2_mass) + sum(Cruise_H2_mass) + sum(Descent_H2_mass) + sum(Taxi_H2_mass)
        H2_mass_array = np.concatenate((Climb_H2_mass, Cruise_H2_mass, Descent_H2_mass, Taxi_H2_mass))
        # Air mass
        Air_flow = max(max(TO_air_massflow), max(Climb_air_massflow), max(Cruise_air_massflow),
                       max(Descent_air_massflow), max(Taxi_air_massflow))

        # Volume of hydrogen for 300 bar, 700 bar and liquid form [m^3]
        H2_volume_300bar = H2_mass * margin / H2_density_300bar
        H2_volume_700bar = H2_mass * margin / H2_density_700bar
        H2_volume_liq = H2_mass * margin / H2_density_liq

        outputs["fuelcell_Pelec_max"] = float(Cruise_Pelec_max)
        outputs["fuelcell_weight"] = float(FC_stack_mass)
        outputs["fuel_consumed_t_econ"] = H2_mass_array
        outputs["data:geometry:propulsion:fuelcell:stacks"] = Cruise_stacks
        outputs["data:geometry:propulsion:fuelcell:weight"] = float(FC_stack_mass)
        outputs["data:geometry:propulsion:fuelcell:volume"] = float(FC_stack_volume)
        outputs["data:geometry:propulsion:hydrogen:weight"] = float(H2_mass)
        outputs["data:geometry:propulsion:fuelcell:air_flow"] = float(Air_flow)
        outputs["data:geometry:propulsion:hydrogen:volume_300bar"] = float(H2_volume_300bar)
        outputs["data:geometry:propulsion:hydrogen:volume_700bar"] = float(H2_volume_700bar)
        outputs["data:geometry:propulsion:hydrogen:volume_liquid"] = float(H2_volume_liq)
