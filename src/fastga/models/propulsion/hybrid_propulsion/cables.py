import math
import fastoad.api as oad
import openmdao.api as om

from fastga.models.propulsion.hybrid_propulsion.constants import SUBMODEL_CABLES_MASS


@oad.RegisterSubmodel(SUBMODEL_CABLES_MASS,
                      "fastga.submodel.propulsion.hybrid_propulsion.cables.legacy")
class ComputeCableMass(om.ExplicitComponent):
    def setup(self):
        self.add_input("motor_efficiency", val=0.85)
        self.add_input("switch_efficiency", val=0.95)
        self.add_input("gearbox_efficiency", val=0.98)
        self.add_input("controller_efficiency", val=0.95)
        self.add_input("bus_efficiency", val=0.95)
        self.add_input("converter_efficiency", val=0.90)
        self.add_input("cables_efficiency", val=0.80)
        self.add_input("rho_aluminium", val=2.8 * 1000)   # [kg/m^3]
        self.add_input("rho_copper", val=8.9 * 1000)   # [kg/m^3]

        self.add_input("data:propulsion:cables:current")
        self.add_input("data:propulsion:cables:voltage")
        self.add_output("data:geometry:propulsion:cables:weight")

    def specificMass(self, inputs, outputs):

        outputs["data:geometry:propulsion:cables:weight"] = 500

    #     total_efficiency = inputs["motor_efficiency"] * inputs["gearbox_efficiency"] * inputs["controller_efficiency"] \
    #                        * inputs["switch_efficiency"] * inputs["bus_efficiency"] * inputs["converter_efficiency"] * \
    #                        inputs["cables_efficiency"]
    #
    #     # Conductor
    #     rho_Al = inputs["rho_aluminium"]
    #     rho_Cu = inputs["rho_copper"]
    #
    #     I = inputs["data:propulsion:cables:current"]
    #     V = inputs["data:propulsion:cables:voltage"]
    #
    #     CabArea_Al = 0.0298 * (I ^ 1.46349) * 1e-6  # [m^2]
    #     CabArea_Cu = 0.0144 * (I ^ 1.4642) * 1e-6  # [m^2]
    #     Cab_radius = math.sqrt(CabArea_Al / math.pi)  # [m]
    #     Cab_circumference = 2 * math.pi * Cab_radius  # [m]
    #
    #     Cab_Length_specific_mass_Al = rho_Al * CabArea_Al  # [kg/m]
    #     Cab_Length_specific_mass_Cu = rho_Cu * CabArea_Cu  # [kg/m]
    #
    #     # Insulation
    #     rho_HEPR = 1.6 * 1000  # [kg/m^3]
    #     Ins_thickness = (0.2325 * V * 1000 + 1.7368) * 0.001  # [m]
    #     Ins_Length_specific_mass = rho_HEPR * Ins_thickness * Cab_circumference  # [kg/m]
    #     Ins_circumference = 2 * math.pi * (Cab_radius + Ins_thickness)
    #
    #     # Sheath
    #     rho_EVA = 0.93 * 1000  # [kg/m^3]
    #     Sht_thickness = (0.035 * (2 * Cab_radius * 1000) + 1) * 0.001  # [m]
    #     Sht_Length_specific_mass = rho_EVA * Sht_thickness * Ins_circumference  # [kg/m]
    #
    #     return Cab_Length_specific_mass_Al, Ins_Length_specific_mass, Sht_Length_specific_mass
    #
    # # Fuel cell to Switch
    # PowerFC
    # VoltageFC
    # CurrentFC = PowerFC / VoltageFC
    # cableLenFC2Switch
    # [Cab_Length_specific_mass_Al, Ins_Length_specific_mass, Sht_Length_specific_mass] \
    #     = specificMass(CurrentFC, VoltageFC)
