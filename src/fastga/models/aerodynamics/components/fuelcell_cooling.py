import math
import numpy as np
import fastoad.api as oad
from openmdao.core.explicitcomponent import ExplicitComponent
from stdatm import Atmosphere

from ..constants import SUBMODEL_FUELCELL_COOLING


@oad.RegisterSubmodel(SUBMODEL_FUELCELL_COOLING, "fastga.submodel.aerodynamics.fuelcell.cooling.legacy")
class Cooling_Airflow(ExplicitComponent):
    """
    Estimation of airflow needed to cool the condenser of the refrigeration cycle
    using the refrigerant R134a.

    Based on : Thermal management system models for overall aircraft design,
    ISAE-SUPAERO, 2021.

    """

    def initialize(self):
        self.options.declare(
            "number_of_points", default=1, desc="number of equilibrium to be treated"
        )

    def setup(self):

        number_of_points = self.options["number_of_points"]

        self.add_input("fuelcell_Pelec_max", val=np.nan, units="W")
        self.add_input("data:propulsion:fuelcell:efficiency", val=0.50)
        self.add_input("altitude", val=np.full(number_of_points, np.nan), units="m")

        self.add_output("data:geometry:propulsion:fuelcell:cooling:airflow", units="kg/s")
        self.add_output("data:geometry:propulsion:fuelcell:cooling:max_airflow")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        altitude = inputs["altitude"]
        T_ain = Atmosphere(altitude, altitude_in_feet=False).temperature

        power = inputs["fuelcell_Pelec_max"]
        fuelcell_efficiency = inputs["data:propulsion:fuelcell:efficiency"]

        heat_dissipated = (1 - fuelcell_efficiency) * power

        # Constants for estimating the specific heat capacity of air
        Cp_0 = 1005.7
        A = 19.48
        B = -0.13158
        C = 0.0019575
        D = -0.000005812
        E = -8.9
        F = 243.5

        Cp = ((Cp_0 + A*T_ain + B*T_ain**2 + C*T_ain**3 + D*T_ain**4) / (
          1 + E/T_ain + F/T_ain**2)
        )

        fuel_cell_effectiveness = 0.8
        T_fin = 383  # temperature in kelvin (110 Celsius based on approximations)

        delta_T = fuel_cell_effectiveness * (T_fin - T_ain)

        airflow = heat_dissipated / (Cp * delta_T)
        max_airflow = np.amax(airflow)
        index = np.where(airflow == max_airflow)[0][0]

        outputs["data:geometry:propulsion:fuelcell:cooling:airflow"] = max_airflow
        outputs["data:geometry:propulsion:fuelcell:cooling:max_airflow"] = index
