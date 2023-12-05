import time
from typing import Tuple

from dt_modeling.odometry.types import Pose2DEstimate
from dt_modeling.odometry.velocity_odometer import VelocityToPose
from .base import Component
from ..types import IQueue, Queue, LambdaQueue, SHUTDOWN_DUMMY

V, Omega, Time = float, float, float


class OdometryComponent(Component[Tuple[V, Omega], Pose2DEstimate]):
    """
    Integrates the velocity of the robot over time in order to continuously obtain a pose estimate that is
    relative to the pose at which the integration started.
    """

    def __init__(self):
        super(OdometryComponent, self).__init__()
        # create odometer
        self._odometer: VelocityToPose = VelocityToPose()
        # queues
        self.in_timed_speed: IQueue[Tuple[V, Omega, Time]] = Queue()
        self.out_pose: IQueue[Pose2DEstimate] = Queue()
        # simpler queue with no time, we add the current time
        self.in_speed: IQueue[Tuple[V, Omega]] = LambdaQueue(
            self.in_timed_speed,
            put=lambda t: t if t is SHUTDOWN_DUMMY else (*t, time.time())
        )

    def worker(self):
        while not self.is_shutdown:
            v, omega, timestamp = self.in_timed_speed.get()
            self._odometer.update(v, omega, timestamp=timestamp)
            pose: Pose2DEstimate = self._odometer.get_estimate() or Pose2DEstimate(0, 0, 0, timestamp)
            self.out_pose.put(pose)
