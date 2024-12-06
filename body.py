from typing import Self
import planetdata
import constants
import kinematicobject


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
    ):
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
