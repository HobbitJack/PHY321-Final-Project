#!/usr/bin/python
# N+M body simulation
import sys

import matplotlib.pyplot
import numpy

import body
import constants


# Procedure
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

    # Function
    @staticmethod
    def object_distance(object1: body.Body, object2: body.Body) -> float:
        return (
            object1.kinematic.position[-1] - object2.kinematic.position[-1]
        ).norm() - (object1.radius + object2.radius)

    # Procedure
    def generate_solar_system(
        self,
        planet_system: list["str"],
        generate_small_objects: bool | int,
        two_dimensions: bool = False,
        generate_rings: bool = False,
    ) -> None:
        for massive_object in planet_system:
            new_object = body.Body.from_table(
                massive_object, self.constants, two_dimensions
            )
            self.massive_bodies.append(new_object)
            self.all_objects.append(new_object)

        for obj_num in range(generate_small_objects):
            self.all_objects.append(
                body.Body.generate_small_body(
                    f"Obj_{obj_num}",
                    self.constants,
                    two_dimensions,
                    (obj_num if generate_rings else -1, generate_small_objects),
                )
            )

    # Procedure
    def __init__(
        self,
        constant: constants.Constants,
        planet_system: list[str],
        small_objects: bool | int,
        two_dimensional: bool = False,
        generate_rings: bool = False,
    ) -> None:
        self.constants = constant
        self.current_time: int = 0  # Seconds
        self.current_timestep: int = 0
        self.massive_bodies: list[body.Body] = []
        self.all_objects: list[body.Body] = []

        self.generate_solar_system(
            planet_system, small_objects, two_dimensional, generate_rings
        )

    # Function
    def check_collision(self, current_body) -> bool:
        for massive_body in self.massive_bodies:
            if current_body is massive_body:
                continue
            if SolarSystem.object_distance(current_body, massive_body) <= 0:
                return True
        return False

    # Procedure
    def update_bodies(self):
        # Calculate initial net force
        for current_body in self.all_objects:
            current_body.kinematic.net_force = current_body.kinematic.compute_net_force(
                [massive_body.kinematic for massive_body in self.massive_bodies]
            )

        # Update position
        for current_body in self.all_objects:
            current_body.kinematic.update_position()
            if len(current_body.kinematic.position) > 500:
                current_body.kinematic.position.pop(0)

        # Calculate new velocities
        for current_body in self.all_objects:
            current_body.kinematic.update_velocity(
                [massive_body.kinematic for massive_body in self.massive_bodies]
            )

    # Procedure
    def collide_bodies(self):
        for index, current_body in enumerate(self.all_objects):
            if self.check_collision(current_body):
                print(f"Body {index} collides!")
                current_body.kinematic.destroyed = True

    # Procedure
    def update_state(self):
        self.current_timestep += 1
        self.current_time += self.constants.delta_time

    # Procedure
    def run_system_to_end(self) -> None:
        while self.current_time <= self.constants.final_time:
            if not self.constants.quiet:
                self.print_progress()
            self.update_bodies()
            self.collide_bodies()
            self.update_state()

    # Procedure
    def print_progress(self) -> None:
        print(
            f"{self.current_timestep / self.constants.total_timesteps * 100:.2f}% computing...",
            file=sys.stderr,
        )

    # Procedure
    def print_body_list(self, filter_list: list[int]) -> None:
        body_list_print = ["BODYLIST"]
        for index, current_body in enumerate(self.all_objects):
            if filter_list and index not in filter_list:
                continue
            body_list_print.append(" ".join([str(index), current_body.name]))
        print("\t".join(body_list_print))

    # Procedure
    def print_moon_detection(
        self, body_types: tuple[list[int], list[int], list[int], list[tuple[int, int]]]
    ) -> None:
        for current_body in body_types[3]:
            if current_body[1] != 0:
                print(
                    f"{self.all_objects[current_body[0]].name} orbits {current_body[1].name}"
                )

    # Function
    def get_resonances(self, current_body: body.Body) -> list[float]:
        resonances = []
        for massive_body in self.massive_bodies:
            resonance = current_body.kinematic.get_orbital_resonance(
                massive_body.kinematic, self.massive_bodies[0].kinematic
            )
            if isinstance(resonance, str):
                resonances.append(resonance)
            else:
                resonances.append(float(resonance))

        return resonances

    def print_resonance(self) -> None:
        resonance_print = ["RESONANCE"]
        for current_body in self.all_objects:
            if current_body.destroyed:
                resonance_print.append("Destroyed")
            resonances = self.get_resonances(current_body)
            resonance_print.append(" ".join([str(res) for res in resonances]))
        print("\t".join(resonance_print))

    # Procedure
    def print_system(
        self, filter_list: bool | list[int], last_state: bool = False
    ) -> None:
        if last_state:
            position_list = ["POSITION"]
            for index, current_body in enumerate(self.all_objects):
                if filter_list and index not in filter_list:
                    continue
                if current_body.destroyed:
                    position_list.append("Destroyed")
                position_list.append(
                    " ".join(
                        [str(coord) for coord in current_body.kinematic.position[-1]]
                    )
                )
            print("\t".join(position_list))
        else:
            for index, time in enumerate(self.constants.time_list):
                current_output_row = []
                for body_index, current_body in enumerate(self.all_objects):
                    if filter_list and body_index not in filter_list:
                        continue
                    if index >= len(current_body.kinematic.position):
                        current_output_row.append("Destroyed")
                        continue
                    current_output_row.append(
                        " ".join(
                            [
                                str(coord)
                                for coord in current_body.kinematic.position[index]
                            ]
                        )
                    )
                if not " ".join(current_output_row).strip():
                    break
                print("\t".join([str(time)] + current_output_row))

        velocity_list = ["VELOCITY"]
        for body_index, current_body in enumerate(self.all_objects):
            if filter_list and body_index not in filter_list:
                continue
            if current_body.kinematic.destroyed:
                velocity_list.append("Destroyed")
                continue
            velocity_list.append(
                " ".join([str(coord) for coord in current_body.kinematic.velocity])
            )
        print("\t".join(velocity_list))

    # Procedure
    def generate_system_plot(
        self, two_dimensions: bool, filter_list: bool | list[int] = False
    ) -> None:
        if two_dimensions:
            matplotlib.pyplot.figure(figsize=(15, 15))
            axes = matplotlib.pyplot.gca()
            axes.set_aspect("equal")

        else:
            axes = matplotlib.pyplot.figure(figsize=(15, 15)).add_subplot(
                projection="3d"
            )

        for index, current_body in enumerate(self.all_objects):
            if filter_list:
                if index not in filter_list and current_body not in self.massive_bodies:
                    continue
            position_x = [position[0] for position in current_body.kinematic.position]
            position_y = [position[1] for position in current_body.kinematic.position]
            if two_dimensions:
                current_color = axes._get_lines.get_next_color()
                matplotlib.pyplot.plot(
                    position_x,
                    position_y,
                    label=f"{current_body.name}",
                    color=current_color,
                )
                if current_body in self.massive_bodies:
                    matplotlib.pyplot.plot(
                        position_x[-1], position_y[-1], "o", color=current_color
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

    # Function
    def compute_body_types(
        self,
    ) -> tuple[list[int], list[int], list[int], list[tuple[int, int]]]:
        degenerate: list[int] = []
        trojans: list[int] = []
        ejected: list[int] = []
        orbitting: list[tuple[int, int]] = []

        for index, current_body in enumerate(self.all_objects):
            focus_object = current_body.kinematic.get_focus_massive_body(
                [massive_body.kinematic for massive_body in self.massive_bodies]
            )
            if focus_object == -1:
                trojans.append(index)
            elif focus_object == "Degenerate":
                degenerate.append(index)
            elif focus_object == "Hyperbola":
                ejected.append(index)
            else:
                orbitting.append((index, focus_object))
        return (degenerate, trojans, ejected, orbitting)
