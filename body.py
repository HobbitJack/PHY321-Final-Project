from typing import Self

import sys
import random
import math

import planetdata
import constants
import kinematicobject
import vector


class Body:
    def __init__(
        self,
        name: str,
        mass: float,
        radius: float,
        position_initial: tuple[float, float, float],
        velocity_initial: tuple[float, float, float],
        constant: constants.Constants,
        is_massive: bool = False,
    ) -> None:
        self.name = name
        self.mass = mass
        self.radius = radius
        self.kinematic = kinematicobject.KinematicObject(
            mass, position_initial, velocity_initial, constant
        )
        self.destroyed = False

    @staticmethod
    def from_table(
        object_name: str, constant: constants.Constants, two_dimension: bool = False
    ) -> Self:
        try:
            planet = planetdata.PLANET_DATA_TABLE[object_name]
        except KeyError as e:
            print(f"body.py: {e.args}: No such body")
            sys.exit(1)
        return Body(
            object_name,
            planet["mass"],
            planet["radius"],
            (
                (planet["x"], planet["y"])
                if two_dimension
                else (planet["x"], planet["y"], planet["z"])
            ),
            (
                (planet["vx"], planet["vy"])
                if two_dimension
                else (planet["vx"], planet["vy"], planet["vz"])
            ),
            constant,
        )

    @staticmethod
    def generate_small_body(
        object_name: str,
        constant: constants.Constants,
        two_dimension: bool = False,
        generate_rings: tuple[int, int] = (-1, -1),  # Current, Total
    ) -> Self:

        if generate_rings[0] == -1:
            position = [
                ((random.random() * 1e9) + 1e7)
                * math.copysign(1, random.random() - 0.5)
                for _ in range(0, 2 + int(not two_dimension))
            ]
        else:
            radius = (
                constant.ring_center
                - (constant.ring_spacing * (constant.num_ring - 1) / 2)
                + (generate_rings[0] % constant.num_ring) * constant.ring_spacing
            )
            per_ring = generate_rings[1] / constant.num_ring
            angle = (2 * math.pi / per_ring) * (
                generate_rings[0]
                - (generate_rings[0] % constant.num_ring) / constant.num_ring
            )
            position = [radius * math.cos(angle), radius * math.sin(angle)] + [
                ((random.random() * 9e5) + 1e5)
                * math.copysign(1, random.random() - 0.5)
                for _ in range(0, not two_dimension)
            ]

        position_vector = vector.Vector(*position)
        distance = position_vector.norm()

        velocity = math.sqrt(
            constant.gravitational_constant
            * planetdata.PLANET_DATA_TABLE["Sun"]["mass"]
            / distance
        )

        velocity_vector = vector.Vector(
            *[
                coord
                for coord in (
                    velocity
                    * vector.Vector(position_vector[0], position_vector[1])
                    .rotate(90)
                    .normalize()
                )
            ]
            + [random.random() for _ in range(0, len(position_vector) - 2)]
        )

        # velocity_vector *= random.random() + 0.5

        return Body(
            object_name,
            constant.small_body_mass,
            constant.small_body_radius,
            position,
            [coord for coord in velocity_vector],
            constant,
        )
