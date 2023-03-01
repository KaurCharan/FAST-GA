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
import logging
_LOGGER = logging.getLogger(__name__)


class BatteryModel:
    def __init__(
            self,
            power,
            time
    ):
        self.power_input = power
        self.time_input = time

    def compute_soc(self):
        _LOGGER.debug("Inside battery model")
        """
        Computation of the SOC.

        :param thrust: Thrust (in N)
        :param engine_setting: Engine settings (climb, cruise,... )
        :param atmosphere: Atmosphere instance at intended altitude
        :return: DOD (in %) and Power (in W)
        """
        global R, C
        rho_cell = 240  # specific energy [Wh/kg]
        vol_elecSys = 500  # system operating voltage [Volt]
        V_cell = 3.3  # single cell nominal voltage [Volt] !!fixed!!
        V_nom = 3.6
        V_cell_max = 4.2  # single cell max voltage [Volt]
        V_cutoff = 2.5  # cut-off voltage [Volt]
        Q_rat = 3  # rated capacity of cell [Ah]
        I_rat = 3  # rated current [Amp]
        I_max = 20  # max continuous current [Amp]
        cell_mass = 44.5 / 1000  # in [kg]
        C_max = 6  # per hour

        Power = self.power_input  # input power profile [Watt] ----------
        t_duration = self.time_input  # input time duration [seconds]

        ## initial voltage estimation of a battery module
        Vb_est = np.zeros(len(Power))  # estimated voltage of a module
        Q_used = np.zeros(len(Power))
        Vc = np.zeros(len(Power))
        Voc = V_cell
        del_V = np.zeros(len(Power))
        del_V[0] = 0
        Ib = np.zeros(len(Power))
        eff_bat = np.zeros(len(Power))
        C_rate = np.zeros(len(Power))
        dod = np.zeros(len(Power))
        dod[0] = 0
        R = 0
        C = 0
        i = 0

        t = t_duration / 60 / 60
        P_in = Power
        n_series = vol_elecSys // V_nom + 1  # number of cells in series in a module
        if i == 0:
            if Power[i] == 0:
                n_parallel = 100

            else:
                n_parallel = (Power[i]) // (Q_rat * vol_elecSys * C_max)  # initial estimate of modules in parallel

        while i < len(t_duration):
            Pb = 0
            P_in[i] = Power[i]
            t[i] = t_duration[i] / 60 / 60    # [h]

            Ib[i] = (P_in[i]) / (n_parallel * vol_elecSys)  # initial estimate of discharge current

            if P_in[i] == 0:
                Q_used[i] = 0
                C = 0
                C_rate[i] = 0
                dod[i] = 0
                eff_bat[i] = 0
                Ib[i] = 0
                if i == 0:
                    Vc[i] = V_cell_max
                else:
                    Vc[i] = Vc[i-1]
                i = i + 1
                continue
            else:
                while abs(Pb - P_in[i]) > 10 ** -5:
                    if P_in[i] == 0:
                        Ib[i] = 0
                        Q_used[i] = 0
                        if i == 0:
                            Vc[i] = V_cell_max
                        else:
                            Vc[i] = Vc[i - 1]
                        dod[i] = 0
                        eff_bat[i] = 0
                        C_rate[i] = 0
                        Vb_est[i] = Vc[i] * n_series
                        i = i + 1
                        Ib[i] = (P_in[i]) / (n_parallel * Vb_est[i - 1])  # estimate of discharge current
                        Vb_est[i] = Vc[i] * n_series
                        continue

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
                    Q_used[i] = Ib[i] * t[i]
                    C = Ib[i] / Q_rat
                    B = p3 * (C ** 3) + p2 * (C ** 2) + p1 * C + p0
                    y = sum(Q_used[0:i])
                    q = K * (Qb / (Qb - y))
                    r = A * 2.718 ** -(B * y)
                    s = R * Ib[i]
                    Vc[i] = Voc - q + r - s  # actual voltage of a cell

                    if Vc[i] <= V_cutoff:
                        n_parallel = n_parallel + 20
                        Vb_est = np.zeros(len(Power))
                        dod = np.zeros(len(Power))
                        eff_bat = np.zeros(len(Power))
                        C_rate = np.zeros(len(Power))
                        Vc = np.zeros(len(Power))
                        Ib = np.zeros(len(Power))
                        Q_used = np.zeros(len(Power))
                        i = 0

                    if P_in[i] == 0:
                        Ib[i] = 0
                        Q_used[i] = 0
                        if i == 0:
                            Vc[i] = V_cell_max
                        else:
                            Vc[i] = Vc[i-1]
                        dod[i] = 0
                        eff_bat[i] = 0
                        C_rate[i] = 0
                        Vb_est[i] = Vc[i] * n_series
                        i = i + 1
                        Ib[i] = (P_in[i]) / (n_parallel * Vb_est[i - 1])  # estimate of discharge current
                        Vb_est[i] = Vc[i] * n_series
                        continue
                    else:
                        if i == 0:
                            Ib[i] = (P_in[i]) / (n_parallel * vol_elecSys)  # estimate of discharge current
                            Qb = Q_rat * ((I_rat / Ib[i]) ** (pc - 1))
                            Q_used[i] = Ib[i] * t[i]
                            C = Ib[i] / Q_rat
                            B = p3 * (C ** 3) + p2 * (C ** 2) + p1 * C + p0
                            y = sum(Q_used[0:i])
                            q = K * (Qb / (Qb - y))
                            r = A * 2.718 ** -(B * y)
                            s = R * Ib[i]
                            Vc[i] = Voc - q + r - s

                        Vb_est[i] = Vc[i] * n_series  # estimated voltage of a module
                        Ib[i] = (P_in[i] / n_parallel) / Vb_est[i]
                        Q_used[i] = Ib[i] * t[i]
                        C = Ib[i] / Q_rat
                        Pb = Vc[i] * Ib[i] * n_series * n_parallel  # calculated power of pack

                Q_used[i] = Ib[i] * t[i]  # [Ah]
                C_rate[i] = C  # discharge rate
                dod[i] = Ib[i] * t[i] * 100 / Q_rat
                eff_bat[i] = 1 - Ib[i] * R / Voc
                i = i + 1

        if n_parallel > 600:
            print("n_parallel greater than 600")
        Q_sum = sum(Q_used)
        weight = n_parallel * n_series * cell_mass
        return weight, n_series, n_parallel, dod, eff_bat, Q_used

    def compute_weight(self, weight_cells):
        """
        Computes weight of installed propulsion (engine, nacelle and propeller) depending on
        maximum power. Uses model described in : Gudmundsson, Snorri. General aviation aircraft
        design: Applied Methods and Procedures. Butterworth-Heinemann, 2013. Equation (6-44)

        """
        BatteryPack_mass = weight_cells + weight_cells * 0.4
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
