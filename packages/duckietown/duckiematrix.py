from typing import Dict, Optional

from dt_duckiematrix_protocols import Matrix
from dt_duckiematrix_protocols.robot import DB21M


class Duckiematrix:
    _engine: Optional[Matrix] = None
    _robots: Dict[str, DB21M] = {}

    @classmethod
    def get_instance(cls, hostname: str = None) -> Matrix:
        # cache first
        if cls._engine is None:
            # create engine to the matrix engine
            cls._engine = Matrix(hostname, auto_commit=True)
        return cls._engine

    @classmethod
    def get_robot(cls, vehicle_name: str, hostname: str = None) -> DB21M:
        # cache first
        if vehicle_name not in cls._robots:
            engine: Matrix = cls.get_instance(hostname)
            # create connection to the vehicle
            cls._robots[vehicle_name] = engine.robots.DB21M(vehicle_name)
        return cls._robots[vehicle_name]
