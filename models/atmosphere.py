# models/atmosphere_imperial.py

class Atmosphere:
    """
    Simple ISA atmosphere model up to 36,089 ft (troposphere) in Imperial units.
    """

    def __init__(self, altitude_ft=0.0):
        self.g0 = 32.17405               # gravity (ft/s²)
        self.R = 1716.49                 # specific gas constant for dry air (ft·lbf/slug·°R)
        self.T0 = 518.67                 # sea level standard temperature (°R = 59°F + 459.67)
        self.P0 = 2116.22                # sea level standard pressure (lb/ft²)
        self.rho0 = 0.0023769            # sea level standard density (slugs/ft³)
        self.lapse_rate = -0.00356616    # temperature lapse rate (°R/ft) = -3.56616 °R per 1000 ft

        self.altitude = altitude_ft
        self.update(altitude_ft)

    def update(self, altitude_ft):
        temp_exp = 1.73 - 0.000157 * altitude_ft
        temp_ratio = 1.0 - self.lapse_rate * altitude_ft / self.T0
        press_ratio = temp_ratio ** temp_exp
        dens_ratio = press_ratio / temp_ratio
        self.rho = dens_ratio * self.rho0

    def get_density(self):
        return self.rho
