from typing import Self

import constants
import vector


class KinematicObject:
    def __init__(
        self,
        mass: float,
        position_initial: tuple[float, float, float],
        velocity_initial: tuple[float, float, float],
        constant: constants.Constants,
    ) -> None:
        self.mass = mass
        self.position: list[vector.Vector] = [vector.Vector(*position_initial)]
        self.velocity: vector.Vector = vector.Vector(*velocity_initial)
        self.net_force = [0, 0, 0]
        self.constants: constants.Constants = constant

    def distance(self, other: Self) -> float:
        return (self.position[-1] - other.position[-1]).norm()

    def compute_net_force(self, massive_body_kinematics: list[Self]) -> vector.Vector:
        net_force = vector.Vector(0, 0, 0)
        for massive_body in massive_body_kinematics:
            if massive_body is self:
                continue
            net_force -= (
                self.constants.gravitational_constant
                * (massive_body.mass * self.mass)
                * (self.position[-1] - massive_body.position[-1]).normalize()
                / (self.distance(massive_body) ** 2)
            )
        return net_force

    def update_position(self) -> None:
        self.position.append(
            self.position[-1]
            + self.velocity * self.constants.delta_time
            + (self.net_force / self.mass) * self.constants.delta_time**2 / 2
        )

    def update_velocity(self, massive_body_kinematics: list[Self]) -> None:
        self.velocity = self.velocity + (
            (
                (
                    (
                        self.net_force / self.mass
                        + self.compute_net_force(massive_body_kinematics) / self.mass
                    )
                    / 2
                )
                * self.constants.delta_time
            )
        )
