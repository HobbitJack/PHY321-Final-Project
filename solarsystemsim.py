#!/usr/bin/python
# N+M body simulation
import getopt
import sys

import matplotlib.pyplot

import constants
import solarsystem


TWO_DIMENSIONS = False
PRINT_SYSTEM = True
PLOT_SYSTEM = False
DISPLAY_LEGEND = True
FILENAME = False
NUM_BODIES = False
SOLAR_SYSTEM = ["Sun", "Jupiter"]

if __name__ == "__main__":
    constant = constants.Constants()

    short_options = "2B:d:f:G:lpPqsS:t:hv"
    long_options = [
        "2-dimensions",
        "small-bodies=",
        "delta-time=",
        "output-file=",
        "gravitational_constant=",
        "no-legend",
        "plot",
        "print-and-plot",
        "quiet",
        "solar-system",
        "select-bodies=",
        "help",
        "version",
    ]
    # Check the user isn't an idiot; then read CLI flags
    try:
        options, positional_arguments = getopt.gnu_getopt(
            sys.argv[1:], short_options, long_options
        )
    except getopt.GetoptError as error:
        print(f"solarsystemsim: '-{error.opt}': Option not recognized")
        sys.exit(1)

    # Parse option flags
    for option in options:
        argument = option[1]
        option = option[0]

        if option == "-h" or option == "--help":
            print("Usage: solarsystemsim [OPTIONS]...")
            print("Simulate a solar system with massive N-bodies and small M-bodies")
            print("Default behavior will print positions to STDOUT")
            print()
            print("System options:")
            print("  -B, --small-bodies=M             Generate M small bodies")
            print("  -s, --solar-system               Use full solar system")
            print("  -S, --select-bodies=BODIES       Use only bodies BODIES")
            print()
            print("Simulation options:")
            print(
                "  -2, --2-dimensions               Ignore Z direction, simulating in 2D"
            )
            print("  -d, --delta-time=DELTATIME       Use DELTATIME as timestep")
            print(
                "  -G, --gravitational-constant=G   Use G in km^3/(kg*s^2) for the Universal Gravitational Constant"
            )
            print("  -t, --timesteps=TIMESTEPS        Use TIMESTEPS timesteps")
            print()
            print("Output options:")
            print(
                "  -f, --output-file=FILE           Output plot to FILE instead of displaying"
            )
            print("  -l, --no-legend                  Do not display legend")
            print("  -p, --plot                       Generate plot instead")
            print(
                "  -P, --print-and-plot             Print data to STDOUT and generate plot"
            )
            print("  -q, --quiet                      Disable progress output")
            print()
            print("Getting help:")
            print("  -h, --help                       Print this help and exit")
            print(
                "  -v, --version                    Print version information and exit"
            )
            print("For full documentation, try 'man solarsystemsim'.")
            sys.exit(0)
        elif option == "-v" or option == "--version":
            print("solarsystemsim v1.1.0")
            print("Copyright C Gabe Almeida, Andy [], and Jack Uteg 2024")
            print("Do not redistribute without written permission.")
            sys.exit(0)
        elif option == "-2" or option == "--2-dimensions":
            TWO_DIMENSIONS = True
        elif option == "-B" or option == "--small-bodies":
            try:
                SMALL_BODIES = int(argument)
            except ValueError:
                print(f"solarsystem: {argument}: Invalid body selection")
                sys.exit(1)
        elif option == "-d" or option == "--delta-time":
            try:
                constant.delta_time = int(argument)
            except ValueError:
                print(f"solarsystem: {argument}: Invalid deltatime selection")
                sys.exit(1)
        elif option == "-f" or option == "--output-file":
            FILENAME = argument
        elif option == "-G" or option == "--gravitational-constant":
            try:
                constant.gravitational_constant = float(argument)
            except ValueError:
                print(f"solarsystem: {argument}: Invalid G selection")
                sys.exit(1)
        elif option == "-l" or option == "--no-legend":
            DISPLAY_LEGEND = False
        elif option == "-p" or option == "--plot":
            PRINT_SYSTEM = False
            PLOT_SYSTEM = True
        elif option == "-P" or option == "--print-and-plot":
            PRINT_SYSTEM = True
            PLOT_SYSTEM = True
        elif option == "-q" or option == "--quiet":
            constant.quiet = True
        elif option == "-s" or option == "--solar-system":
            SOLAR_SYSTEM = solarsystem.SolarSystem.SOLAR_SYSTEM_PLANETS
        elif option == "-S" or option == "--select-bodies":
            SOLAR_SYSTEM = [
                solar_system_object.strip().title()
                for solar_system_object in argument.split(",")
            ]
        elif option == "-t" or option == "--timesteps":
            try:
                constant.total_timesteps = int(argument)
            except ValueError:
                print(f"solarsystem: {argument}: Invalid timesteps selection")
                sys.exit(1)

    constant.update_values()
    system = solarsystem.SolarSystem(constant, SOLAR_SYSTEM, NUM_BODIES, TWO_DIMENSIONS)

    system.run_system_to_end()

    if PRINT_SYSTEM:
        system.print_header()
        system.print_system()

    if PLOT_SYSTEM:
        system.generate_system_plot(TWO_DIMENSIONS)

        if DISPLAY_LEGEND:
            matplotlib.pyplot.legend()

        if FILENAME:
            matplotlib.pyplot.savefig(fname=f"{FILENAME.removesuffix('.png')}.png")
        else:
            matplotlib.pyplot.show()
