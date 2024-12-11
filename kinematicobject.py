from typing import Self

import numpy

import constants
import vector


class KinematicObject:
    # Procedure
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
        self.destroyed = False

    # Function
    def distance(self, other: Self) -> float:
        return (self.position[-1] - other.position[-1]).norm()

    # Function
    def compute_net_force(self, massive_body_kinematics: list[Self]) -> vector.Vector:
        if self.destroyed:
            return False
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

    # Procedure
    def update_position(self) -> None:
        if self.destroyed:
            return
        self.position.append(
            self.position[-1]
            + self.velocity * self.constants.delta_time
            + (self.net_force / self.mass) * self.constants.delta_time**2 / 2
        )

    # Procedure
    def update_velocity(self, massive_body_kinematics: list[Self]) -> None:
        if self.destroyed:
            return
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

    # Function
    def get_orbit(self) -> tuple[float, float, float, float, float, float]:
        if not self.destroyed:
            return get_conic_section(
                [self.position[index] for index in range(-50, 1, 10)]
            )  # Use last 30 days
        return "Degenerate"

    def get_orbit_standard_form(
        self,
    ) -> tuple[float, float, float, float] | str:
        orbit_conic = self.get_orbit()
        if orbit_conic == "Degenerate":
            return orbit_conic
        orbit_type = classify_conic(*orbit_conic)
        if orbit_type != "Ellipse" and orbit_type != "Circle":
            return orbit_type

        return get_standard_form(*orbit_conic)

    # Function
    def get_focus_massive_body(self, massive_body_kinematics: list[Self]) -> int:
        # Returns index of focus massive body
        # None if degenerate, False if not ellipse, and -1 if no body at focus

        orbit = self.get_orbit_standard_form()

        if isinstance(orbit, str):
            return orbit

        ellipse_foci = get_ellipse_foci(orbit)

        foci_distances = []
        for massive_body in massive_body_kinematics:
            if get_inside_ellipse(massive_body.position[-1], orbit):
                foci_distances.append(
                    (
                        (ellipse_foci[0] - massive_body.position[-1]).norm(),
                        (ellipse_foci[1] - massive_body.position[-1]).norm(),
                    )
                )

        if not foci_distances:
            return -1

        min_distance = min(foci_distances[0][0], foci_distances[0][1])
        min_index = 0
        for index, distances in enumerate(foci_distances[1:]):
            if distances[0] < min_distance:
                min_distance = distances[0]
                min_index = index
            if distances[1] < min_distance:
                min_distance = distances[1]
                min_index = index
        return min_index

    # Function
    def get_orbital_period(self, massive_body: Self) -> float:
        orbit = self.get_orbit_standard_form()

        if isinstance(orbit, str):
            return orbit

        return numpy.sqrt(
            numpy.sqrt(numpy.abs(orbit[0])) ** 3
            * 4
            * numpy.pi**2
            / (self.constants.gravitational_constant * (massive_body.mass + self.mass))
        )

    # Function
    def get_orbital_resonance(self, massive_body: Self, parent_body: Self) -> float:
        self_period = self.get_orbital_period(parent_body)
        other_period = massive_body.get_orbital_period(parent_body)
        if not isinstance(self_period, float) or not isinstance(other_period, float):
            return self_period
        return self_period / other_period


# Function
def get_conic_section(
    points: list[vector.Vector], f=1e20
) -> tuple[float, float, float, float, float, float]:
    """Solve for the coefficients of a conic given five points in Numpy array

    `points` should have at least five rows.

    `f` is the constant that you can specify. With the returned solution,
    `(a, b, c, d, e, f)`, the full conic is specified as:

    $a x^2 + b x y + c y^2 + d x + e y = -f$

    If `points` has exactly five rows, the equation will be exact. If `points`
    has *more* than five rows, the solution will be a least-squares one that
    fits the data the best.

    Top answer to Stack Overflow Q 43624502, used under CC BY-SA 3.0
    """
    from numpy.linalg import lstsq

    x_values = numpy.array([point[0] for point in points])
    y_values = numpy.array([point[1] for point in points])
    if len(x_values) < 5:
        raise ValueError("Need >= 5 points to solve for conic section")

    A = numpy.vstack(
        [
            x_values**2,
            x_values * y_values,
            y_values**2,
            x_values,
            y_values,
        ]
    ).T
    fullSolution = lstsq(A, f * numpy.ones(len(x_values)))
    (a, b, c, d, e) = fullSolution[0]
    return (a, b, c, d, e, f)


# Function
def classify_conic(a, b, c, d, e, f) -> str:
    if numpy.isclose((a * c - b**2 / 4) * f + (b * e * d - c * d**2 - a * e**2) / 4, 0):
        return "Degenerate"

    if numpy.sign(a) == numpy.sign(c):
        if numpy.isclose(a, c):
            return "Circle"
        return "Ellipse"

    if a == 0 or c == 0:
        return "Parabola"
    return "Hyperbola"


# Function
def get_standard_form(a, b, c, d, e, f) -> tuple[float, float, float, float]:
    # Returns the canonical form u^2/a^2 + v^2/b^2 = 1
    # for u = cos*x-sin*y; v = sin*y+cos*x
    # [a^2, b^2, sin, cos]
    # Based on top answer to Mathematics Stack Exchange Q 596016, used under CC BY-SA 4.0

    A = a * (a * c - b**2 / 4)
    B = (b / 2) * (a * c - b**2 / 4)
    C = c * (a * c - b**2 / 4)
    F = f * (a * c - b**2 / 4) - a * e**2 / 4 - c * d**2 / 4 + b * d * e / 4

    M = A**2 + C**2 + 4 * B**2 - 2 * A * C

    cos_sq = ((C - A) * numpy.sqrt(M) + M) / (2 * M)
    sin_sq = ((A - C) * numpy.sqrt(M) + M) / (2 * M)
    cos = numpy.sqrt(cos_sq)
    sin = numpy.sqrt(sin_sq)

    return [
        (A * cos_sq - 2 * B * cos * sin + C * sin_sq) / (-F),
        (A * sin_sq + 2 * B * cos * sin + C * cos_sq) / (-F),
        sin,
        cos,
    ]


# Function
def get_inside_ellipse(
    point: vector.Vector, parameters: tuple[float, float, float, float]
) -> bool:
    a_sq, b_sq, sin, cos = parameters
    u = cos * point[0] - sin * point[1]
    v = cos * point[0] + sin * point[1]

    return u**2 / a_sq + v**2 / b_sq < 1


# Function
def get_ellipse_foci(
    parameters: tuple[float, float, float, float]
) -> tuple[vector.Vector, vector.Vector]:
    a_sq, b_sq, sin, cos = parameters
    if numpy.isclose(a_sq, b_sq):
        return vector.Vector(0, 0)
    u = numpy.sqrt(a_sq - b_sq)

    return (
        vector.Vector(u / (2 * cos), -u / (2 * sin)),
        vector.Vector(-u / (2 * cos), u / (2 * sin)),
    )
