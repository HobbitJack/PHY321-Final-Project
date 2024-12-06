#!/usr/bin/python
# N+M body simulation
import getopt
import sys

import matplotlib.pyplot
import numpy

import body
import constants
import kinematicobject
import vector

TWO_DIMENSIONS = False
PRINT_SYSTEM = True
PLOT_SYSTEM = False
DISPLAY_LEGEND = True
FILENAME = False
NUM_BODIES = False
QUIET = False
SOLAR_SYSTEM = ["Sun", "Jupiter"]


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().

    Top answer to StackOverflow Q 13,685,386, used under CC BY-SA 4.0
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = numpy.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = numpy.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = numpy.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


class SolarSystem:
    # Using Comet Hale-Bopp as a guide
    SMALL_BODY_MASS = 1e16  # kg
    SMALL_BODY_RADIUS = 50  # km

    SOLAR_SYSTEM_PLANETS = [
        "Sun",
        "Mercury",
        "Venus",
        "Earth",
        "Mars",
        "Jupiter",
        "Saturn",
        "Uranus",
        "Neptune",
    ]

    @classmethod
    def object_distance(object1: body.Body, object2: body.Body) -> float:
        return (
            object1.kinematic.position[-1] - object2.kinematic.position[-1]
        ).norm() - (object1.radius + object2.radius)

    def generate_solar_system(
        self,
        planet_system: list["str"],
        generate_small_objects: bool,
    ) -> None:
        for massive_object in planet_system:
            new_object = body.Body.from_table(massive_object, constant, TWO_DIMENSIONS)
            self.massive_bodies.append(new_object)
            self.all_objects.append(new_object)

    def __init__(self, planet_system: list[str], small_objects: bool) -> None:
        self.current_time: int = 0  # Seconds
        self.current_timestep: int = 0
        self.massive_bodies: list[body.Body] = []
        self.all_objects: list[body.Body] = []

        self.generate_solar_system(planet_system, small_objects)

    def check_collision(self, body) -> bool:
        for massive_body in self.massive_bodies:
            if massive_body is body.kinematic:
                continue
            if object_distance(body, massive_body) <= 0:
                return True
        return False

    def update_bodies(self):
        # Calculate initial net force
        for current_body in self.all_objects:
            current_body.kinematic.net_force = current_body.kinematic.compute_net_force(
                [massive_body.kinematic for massive_body in self.massive_bodies]
            )

        # Update position
        for current_body in self.all_objects:
            current_body.kinematic.update_position()

        # Calculate new velocities
        for current_body in self.all_objects:
            current_body.kinematic.update_velocity(
                [massive_body.kinematic for massive_body in self.massive_bodies]
            )

    def update_state(self):
        self.current_timestep += 1
        self.current_time += constant.delta_time

    def run_system_to_end(self):
        while self.current_time <= constant.final_time:
            if not QUIET:
                self.print_progress()
            self.update_bodies()
            self.update_state()

    def print_progress(self):
        print(
            f"{self.current_timestep / constant.total_timesteps * 100:.2f}% computing...",
            file=sys.stderr,
        )

    def print_header(self):
        print(
            "\t".join(
                ["TIME"]
                + [
                    "\t".join(["X", "Y"] if TWO_DIMENSIONS else ["X", "Y", "Z"])
                    for body in self.all_objects
                ]
            )
        )

    def print_system(self):
        for index, time in enumerate(constant.time_list):
            print(
                "\t".join(
                    [str(time)]
                    + [
                        str(coord)
                        for current_body in self.all_objects
                        for coord in current_body.kinematic.position[index]
                    ]
                )
            )

    def generate_system_plot(self):
        if TWO_DIMENSIONS:
            matplotlib.pyplot.figure(figsize=(15, 15))
            matplotlib.pyplot.axes().set_aspect("equal")

        else:
            axes = matplotlib.pyplot.figure(figsize=(15, 15)).add_subplot(
                projection="3d"
            )

        for index, body in enumerate(self.all_objects):
            position_x = [position[0] for position in body.kinematic.position]
            position_y = [position[1] for position in body.kinematic.position]
            if TWO_DIMENSIONS:
                matplotlib.pyplot.plot(
                    position_x,
                    position_y,
                    label=f"{body.name}",
                )
            else:
                position_z = [position[2] for position in body.kinematic.position]
                matplotlib.pyplot.plot(
                    position_x,
                    position_y,
                    position_z,
                    label=f"{body.name}",
                )

        matplotlib.pyplot.xlabel("X Position (km)")
        matplotlib.pyplot.ylabel("Y Position (km)")

        if DISPLAY_LEGEND:
            matplotlib.pyplot.legend()

        if not TWO_DIMENSIONS:
            axes.set_box_aspect([1.0, 1.0, 1.0])
            axes.set_zlabel("Z Position (km)")
            set_axes_equal(axes)


if __name__ == "__main__":
    constant = constants.Constants()

    short_options = "2B:d:f:G:lpPqsS:t:hv"
    long_options = [
        "2-dimensions",
        "small-bodies=",
        "delta-time=",
        "output-file=",
        "gravitational_constant=",
        "no-legend" "plot",
        "print-and-plot",
        "quiet" "solar-system",
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
        print(f"solarsystem: '-{error.opt}': Option not recognized")
        sys.exit(1)

    # Parse option flags
    for option in options:
        argument = option[1]
        option = option[0]

        if option == "-h" or option == "--help":
            print("Usage: solarsystem [OPTIONS]...")
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
            print("For full documentation, try 'man solarsystem'.")
            sys.exit(0)
        elif option == "-v" or option == "--version":
            print("solarsystem v1.0.1")
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
            QUIET = True
        elif option == "-s" or option == "--solar-system":
            SOLAR_SYSTEM = SolarSystem.SOLAR_SYSTEM_PLANETS
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
    system = SolarSystem(SOLAR_SYSTEM, NUM_BODIES)

    system.run_system_to_end()

    if PRINT_SYSTEM:
        system.print_header()
        system.print_system()

    if PLOT_SYSTEM:
        system.generate_system_plot()

        if FILENAME:
            matplotlib.pyplot.savefig(fname=f"{FILENAME.removesuffix('.png')}.png")
        else:
            matplotlib.pyplot.show()
