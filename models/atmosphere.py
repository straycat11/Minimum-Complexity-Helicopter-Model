# models/atmosphere.py

class Atmosphere:
    """
    Simple ISA atmosphere model up to 11 km (troposphere).
    """

    def __init__(self, altitude_m=0.0):
        self.g0 = 9.80665            # gravity (m/s²)
        self.R = 287.05              # specific gas constant for dry air (J/kg·K)
        self.T0 = 288.15             # sea level standard temperature (K)
        self.P0 = 101325.0           # sea level standard pressure (Pa)
        self.rho0 = 1.225            # sea level standard density (kg/m³)
        self.lapse_rate = -0.0065   # temperature lapse rate (K/m)

        self.altitude = altitude_m
        self.update(altitude_m)

    def update(self, altitude_m):
        temp_exp = 1.73 - 0.000157 * altitude_m
        temp_ratio = 1.0-self.lapse_rate*(altitude_m)
        press_ratio = temp_ratio**temp_exp
        dens_ratio = press_ratio/temp_ratio
        self.rho = dens_ratio*self.rho0

    # def get_temperature(self):
    #     return self.T

    # def get_pressure(self):
    #     return self.P

    def get_density(self):
        return self.rho
