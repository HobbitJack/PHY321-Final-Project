#!/usr/bin/python
# N+M body simulation

import matplotlib.pyplot

import constants
import solarsystem


PRINT_LASTSTATE = False
PLOT_SYSTEM = False
PLOT_CONIC = False
DISPLAY_LEGEND = True
FILENAME = False
NUM_BODIES = False
GENERATE_RINGS = False
FILTER_OBJECTS = False
SOLAR_SYSTEM = ["Sun", "Jupiter"]
USE_INPUT_DATA = False


# Procedure
def n_body_problem(
    *bodies: str, show_legend: bool = True, show_plot: bool = True
) -> None:
    constant = constants.Constants()
    constant.quiet = True
    constant.update_values()
    system = solarsystem.SolarSystem(
        constant, [current_body for current_body in bodies], 0, True, False
    )

    system.run_system_to_end()

    system.generate_system_plot(True, FILTER_OBJECTS, PLOT_CONIC)

    if show_legend:
        matplotlib.pyplot.legend()

    if show_plot:
        matplotlib.pyplot.show()
