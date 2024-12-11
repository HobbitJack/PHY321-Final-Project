GRAVITATIONAL_CONSTANT = 6.67430e-20  # value in km^3/(kg*s^2)
SECONDS_IN_DAY = 86400
ONE_YEAR = 365
HALE_BOPP_MASS = 1e16
HALE_BOPP_RADIUS = 50
JUPITER_SMA = 7.78479e8
ASTEROID_BELT_SMA = 4.9053e8


class Constants:
    def __init__(self):
        self.gravitational_constant = GRAVITATIONAL_CONSTANT
        self.delta_time = SECONDS_IN_DAY
        self.total_timesteps = ONE_YEAR
        self.quiet = False
        self.small_body_mass = HALE_BOPP_MASS
        self.small_body_radius = HALE_BOPP_RADIUS
        self.num_ring = 6
        self.ring_center = JUPITER_SMA
        self.ring_spacing = 5e7
        self.update_values()

    def update_values(self):
        self.final_time = self.delta_time * self.total_timesteps
        self.time_list = range(0, self.final_time, self.delta_time)
