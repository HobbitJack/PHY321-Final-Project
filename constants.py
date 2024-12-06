GRAVITATIONAL_CONSTANT = 6.67430e-20  # value in km^3/(kg*s^2)
SECONDS_IN_DAY = 86400
ONE_YEAR = 365


class Constants:
    def __init__(self):
        self.gravitational_constant = GRAVITATIONAL_CONSTANT
        self.delta_time = SECONDS_IN_DAY
        self.total_timesteps = ONE_YEAR
        self.update_values()

    def update_values(self):
        self.final_time = self.delta_time * self.total_timesteps
        self.time_list = range(0, self.final_time, self.delta_time)
