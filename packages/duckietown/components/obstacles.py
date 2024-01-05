from typing import Dict, Optional

from duckietown.types import Queue

from .base import Component
from ..types import Queue, DetectedLines, ColorName


class ObstacleAvoidanceComponent(Component[Dict[ColorName, DetectedLines], bool]):
    """
    The Obstacle Avoidance component stops the robot when an obstacle is detected within a certain distance.

    Args:
        TODO

    """

    def __init__(self, stop_distance: float = 0.25):
        super(ObstacleAvoidanceComponent, self).__init__()
        # store arguments
        self._stop_distance = stop_distance
        # queues
        self.in_range: Queue[Optional[float]] = Queue()
        self.out_obstacle: Queue[bool] = Queue()

    def worker(self):
        while not self.is_shutdown:
            range: Optional[float] = self.in_range.get()
            stop: bool = range is not None and range > 0 and range < self._stop_distance
            self.out_obstacle.put(stop)
