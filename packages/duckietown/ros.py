from typing import Dict

import roslibpy


class ROS:
    _connections: Dict[str, roslibpy.Ros] = {}

    @classmethod
    def get_connection(cls, vehicle_name: str, port: int = 9001) -> roslibpy.Ros:
        hostname: str = vehicle_name if vehicle_name.endswith(".local") else f"{vehicle_name}.local"
        if hostname in cls._connections:
            return cls._connections[hostname]
        new: roslibpy.Ros = roslibpy.Ros(host=hostname, port=port)
        cls._connections[hostname] = new
        return new
