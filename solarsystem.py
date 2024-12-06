#!/usr/bin/python
# N+M body simulation
import sys

import matplotlib.pyplot
import numpy

import body
import constants


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

    @staticmethod
    def object_distance(object1: body.Body, object2: body.Body) -> float:
        return (
            object1.kinematic.position[-1] - object2.kinematic.position[-1]
        ).norm() - (object1.radius + object2.radius)

    def generate_solar_system(
        self,
        planet_system: list["str"],
        generate_small_objects: bool,
        two_dimensions: bool = False,
    ) -> None:
        for massive_object in planet_system:
            new_object = body.Body.from_table(
                massive_object, self.constants, two_dimensions
            )
            self.massive_bodies.append(new_object)
            self.all_objects.append(new_object)

    def __init__(
        self,
        constant: constants.Constants,
        planet_system: list[str],
        small_objects: bool,
        two_dimensional: bool = False,
    ) -> None:
        self.constants = constant
        self.current_time: int = 0  # Seconds
        self.current_timestep: int = 0
        self.massive_bodies: list[body.Body] = []
        self.all_objects: list[body.Body] = []

        self.generate_solar_system(planet_system, small_objects, two_dimensional)

    def check_collision(self, current_body) -> bool:
        for massive_body in self.massive_bodies:
            if current_body is massive_body:
                continue
            if SolarSystem.object_distance(current_body, massive_body) <= 0:
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

    def collide_bodies(self):
        for current_body in self.all_objects:
            if self.check_collision(current_body):
                current_body.kinematic.destroyed = True

    def update_state(self):
        self.current_timestep += 1
        self.current_time += self.constants.delta_time

    def run_system_to_end(self):
        while self.current_time <= self.constants.final_time:
            if not self.constants.quiet:
                self.print_progress()
            self.update_bodies()
            self.collide_bodies()
            self.update_state()

    def print_progress(self):
        print(
            f"{self.current_timestep / self.constants.total_timesteps * 100:.2f}% computing...",
            file=sys.stderr,
        )

    def print_header(self, two_dimensions: bool = False):
        print(
            "\t".join(
                ["TIME"]
                + [
                    "\t".join(
                        [f"{current_body.name} X", f"{current_body.name} Y"]
                        if two_dimensions
                        else [
                            f"{current_body.name} X",
                            f"{current_body.name} Y",
                            f"{current_body.name} Z",
                        ]
                    )
                    for current_body in self.all_objects
                ]
            )
        )

    def print_system(self):
        for index, time in enumerate(self.constants.time_list):
            print(
                "\t".join(
                    [str(time)]
                    + [
                        (
                            str(coord)
                            if index < len(current_body.kinematic.position)
                            else ""
                        )
                        for current_body in self.all_objects
                        for coord in current_body.kinematic.position[index]
                    ]
                )
            )

    def generate_system_plot(self, two_dimensions: bool):
        if two_dimensions:
            matplotlib.pyplot.figure(figsize=(15, 15))
            matplotlib.pyplot.axes().set_aspect("equal")

        else:
            axes = matplotlib.pyplot.figure(figsize=(15, 15)).add_subplot(
                projection="3d"
            )

        for index, current_body in enumerate(self.all_objects):
            position_x = [position[0] for position in current_body.kinematic.position]
            position_y = [position[1] for position in current_body.kinematic.position]
            if two_dimensions:
                matplotlib.pyplot.plot(
                    position_x,
                    position_y,
                    label=f"{current_body.name}",
                )
            else:
                position_z = [
                    position[2] for position in current_body.kinematic.position
                ]
                matplotlib.pyplot.plot(
                    position_x,
                    position_y,
                    position_z,
                    label=f"{current_body.name}",
                )

        matplotlib.pyplot.xlabel("X Position (km)")
        matplotlib.pyplot.ylabel("Y Position (km)")

        if not two_dimensions:
            axes.set_box_aspect([1.0, 1.0, 1.0])
            axes.set_zlabel("Z Position (km)")
            set_axes_equal(axes)
