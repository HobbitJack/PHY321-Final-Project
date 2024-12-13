#!/usr/bin/python
# N+M body simulation
import math
import getopt
import sys

import matplotlib.pyplot

import constants
import solarsystem


TWO_DIMENSIONS = False
PRINT_BODYLIST = False
PRINT_RESONANCES = False
PRINT_SYSTEM = False
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

if __name__ == "__main__":
    constant = constants.Constants()

    short_options = "2B:C:d:f:FG:IlLpPrRqsS:t:hv"
    long_options = [
        "2-dimensions",
        "small-bodies=",
        "conic-section=",
        "delta-time=",
        "output-file=",
        "filter-interesting",
        "gravitational_constant=",
        "use-input-data",
        "no-legend",
        "last-state",
        "print",
        "plot",
        "quiet",
        "resonances",
        "rings",
        "solar-system",
        "select-bodies=",
        "timesteps=",
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
            print("  -I, --use-input-data             Read state data from stdin")
            print(
                "                                   Replaces positions and velocites after generation"
            )
            print("  -R, --rings                      Generate small bodies in rings")
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
            print("  -b, --body-list                  Output list of bodies")
            print(
                "  -C, --conic-section=NUM          Plot the conic for body number NUM"
            )
            print(
                "  -f, --output-file=FILE           Output plot to FILE instead of displaying"
            )
            print(
                "  -F, --filter-interesting         Only output for objects identified as interesting"
            )
            print("  -l, --no-legend                  Do not display legend")
            print("  -L, --last-state                 Only print last timestep state")
            print("  -p, --print                      Print columnated position data")
            print("  -P, --plot                       Generate plot with matplotlib")
            print("  -q, --quiet                      Disable progress output")
            print(
                "  -R, --resonances                 Print orbital resonances with each massive body"
            )
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
                NUM_BODIES = int(argument)
            except ValueError:
                print(f"solarsystem: {argument}: Invalid body selection")
                sys.exit(1)
        elif option == "-C" or option == "--conic-section":
            try:
                PLOT_CONIC = int(argument)
            except ValueError:
                print(f"solarsystem: {argument}: Invalid body number")
                sys.exit(1)
        elif option == "-d" or option == "--delta-time":
            try:
                constant.delta_time = int(argument)
            except ValueError:
                print(f"solarsystem: {argument}: Invalid deltatime selection")
                sys.exit(1)
        elif option == "-f" or option == "--output-file":
            FILENAME = argument
        elif option == "-F" or option == "--filter-interesting":
            FILTER_OBJECTS = True
        elif option == "-G" or option == "--gravitational-constant":
            try:
                constant.gravitational_constant = float(argument)
            except ValueError:
                print(f"solarsystem: {argument}: Invalid G selection")
                sys.exit(1)
        elif option == "-I" or option == "--use-input-data":
            USE_INPUT_DATA = True
        elif option == "-l" or option == "--no-legend":
            DISPLAY_LEGEND = False
        elif option == "-L" or option == "--last-state":
            PRINT_LASTSTATE = True
        elif option == "-p" or option == "--plot":
            PRINT_SYSTEM = True
        elif option == "-P" or option == "--print-and-plot":
            PLOT_SYSTEM = True
        elif option == "-q" or option == "--quiet":
            constant.quiet = True
        elif option == "-r" or option == "--resonances":
            PRINT_RESONANCES = True
        elif option == "-R" or option == "--rings":
            GENERATE_RINGS = True
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
    system = solarsystem.SolarSystem(
        constant, SOLAR_SYSTEM, NUM_BODIES, TWO_DIMENSIONS, GENERATE_RINGS
    )

    if USE_INPUT_DATA:
        import vector

        state_data = sys.stdin.read().splitlines()
        position = state_data[-2].split("\t")[1:]
        velocity = state_data[-1].split("\t")[1:]
        for index, current_body in enumerate(system.all_objects):
            if index >= len(position) or index >= len(velocity):
                break
            if position[index] == "Destroyed":
                current_body.kinematic.destroyed = True
                current_body.destroyed = True
            current_body.kinematic.position = [
                vector.Vector(*[float(coord) for coord in position[index].split(" ")])
            ]
            current_body.kinematic.velocity = vector.Vector(
                *[float(coord) for coord in velocity[index].split(" ")]
            )

    system.run_system_to_end()
    body_types = system.compute_body_types()

    if PRINT_SYSTEM:
        system.print_moon_detection(body_types)

        if PRINT_BODYLIST:
            system.print_body_list(FILTER_OBJECTS)

        if PRINT_RESONANCES:
            system.print_resonance()

        system.print_system(FILTER_OBJECTS, PRINT_LASTSTATE)

    if PLOT_SYSTEM:
        system.generate_system_plot(TWO_DIMENSIONS, FILTER_OBJECTS, PLOT_CONIC)

        if DISPLAY_LEGEND:
            matplotlib.pyplot.legend()

        if FILENAME:
            matplotlib.pyplot.savefig(fname=f"{FILENAME.removesuffix('.png')}.png")
        else:
            matplotlib.pyplot.show()
