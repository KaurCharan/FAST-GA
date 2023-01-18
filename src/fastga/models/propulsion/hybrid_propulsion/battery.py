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

import numpy as np


class BatteryModel:
    def __init__(
            self,
            power,
            time
    ):
        self.power_input = power
        self.time_input = time

    def compute_soc(self):
        """
        Computation of the SOC.

        :param thrust: Thrust (in N)
        :param engine_setting: Engine settings (climb, cruise,... )
        :param atmosphere: Atmosphere instance at intended altitude
        :return: SOC (in %) and Power (in W)
        """
        global R, C
        rho_cell = 240  # specific energy [Wh/kg]
        vol_elecSys = 500  # system operating voltage [Volt]
        V_cell = 3.3  # single cell nominal voltage [Volt] !!fixed!!
        V_nom = 3.6
        V_cell_max = 4.2  # single cell max voltage [Volt]
        V_cutoff = 2.6  # cut-off voltage [Volt]
        Q_rat = 3  # rated capacity of cell [Ah]
        I_rat = 3  # rated current [Amp]
        I_max = 20  # max continuous current [Amp]
        cell_mass = 44.5 / 1000  # in [kg]
        C_max = 6  # per hour

        Power = self.power_input  # input power profile [Watt] ----------
        t_duration = self.time_input  # input time duration [seconds]

        ## initial voltage estimation of a battery module
        # Q_pack = W_total * rho_bat / n_pack / vol_elecSys   # Capacity of battery pack
        Vb_est = list(range(len(Power)))  # estimated voltage of a module
        Q_used = list(range(len(Power)))
        Vc = list(range(len(Power)))
        Voc = V_cell
        del_V = list(range(len(Power)))
        P_in = list(range(len(Power)))
        t = list(range(len(Power)))
        del_V[0 - 1] = 0
        Ib = list(range(len(Power)))
        eff_bat = list(range(len(Power)))
        C_rate = list(range(len(Power)))
        soc = list(range(len(Power)))
        soc[0 - 1] = 100
        i = 0

        n_series = vol_elecSys // V_nom + 1  # number of cells in series in a module
        if i == 0:
            n_parallel = (Power[i]) // (Q_rat * vol_elecSys * C_max)  # initial estimate of modules in parallel
            print("initial value of n_parallel", n_parallel)

        while i < len(t_duration):
            print(i)
            Pb = 0
            P_in[i] = Power[i]
            t[i] = t_duration[i] / 60 / 60
            print("power input", P_in[i])
            Ib[i] = (P_in[i]) / (n_parallel * vol_elecSys)  # initial estimate of discharge current
            print("Ib is", Ib[i])
            while abs(Pb - P_in[i]) > 10 ** -5:
                ## BATTERY MODEL
                pc = 1.0085
                p3 = 0.003
                p2 = -0.048
                p1 = 0.176
                p0 = 0.524
                R = 0.026
                A = 0.91
                K = 0.0165

                Qb = Q_rat * ((I_rat / Ib[i]) ** (pc - 1))
                C = Ib[i] / Q_rat
                B = p3 * (C ** 3) + p2 * (C ** 2) + p1 * C + p0
                q = K * (Qb / (Qb - Ib[i] * t[i]))
                r = A * 2.718 ** -(B * Ib[i] * t[i])
                s = R * Ib[i]
                Vc[i] = Voc - q + r - s - del_V[i - 1]  # actual voltage of a cell
                Pb = Vc[i] * Ib[i] * n_series * n_parallel  # calculated power of pack

                if Vc[i] <= V_cutoff:
                    n_parallel = n_parallel + 20
                    del_V[i] = 0
                    i = 0
                    print("n-parallel", n_parallel)
                    print("new value of i", i)

                Vb_est[i] = Vc[i] * n_series  # estimated voltage of a module
                Ib[i] = (P_in[i] / n_parallel) / Vb_est[i]
                Q_used[i] = Ib[i] * t[i]

                if n_parallel > 600:
                    print("n_parallel greater than 600")
                    break

            del_V[i] = 4.2 - Vc[i]
            Q_used[i] = Ib[i] * t[i] + Q_used[i - 1]
            # Q_remain = Q_rat - Q_used[i]
            C_rate[i] = C  # discharge rate
            print("soc is", soc[i - 1])
            print("capacity used is", Ib[i] * t[i] * 100 / Q_rat)
            soc[i] = soc[i - 1] - Ib[i] * t[i] * 100 / Q_rat
            print("new soc is", soc[i])
            eff_bat[i] = 1 - Ib[i] * R / Voc
            i = i + 1

        # n = Q_remain / Q_rat
        weight = n_parallel * n_series * cell_mass
        return weight, n_series, n_parallel, soc, eff_bat, C_rate

    def compute_weight(self, weight_cells):
        """
        Computes weight of installed propulsion (engine, nacelle and propeller) depending on
        maximum power. Uses model described in : Gudmundsson, Snorri. General aviation aircraft
        design: Applied Methods and Procedures. Butterworth-Heinemann, 2013. Equation (6-44)

        """
        BatteryPack_mass = weight_cells + weight_cells * 0.6
        return BatteryPack_mass

    def compute_volume(self, n_parallel, n_series):
        """
        add

        """
        dia = 18.3  # mm
        length = 65  # mm

        # Compute battery pack volume
        BatteryPack_volume = (np.pi * (dia ** 2) / 4 / 0.785) * length * n_parallel * n_series / 0.6
        return BatteryPack_volume
