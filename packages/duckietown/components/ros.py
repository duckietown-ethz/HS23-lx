import base64
from abc import abstractmethod
from typing import Optional, Any

import roslibpy
from turbojpeg import TurboJPEG

from .base import Component, OutputType
from ..ros import ROS
from ..types import JPEGImage, BGRImage, Queue

Range = float

__all__ = [
    "CameraDriverComponent",
    "TimeOfFlightDriverComponent"
]


class GenericROSSubscriberComponent(Component[None, OutputType]):

    def __init__(self, vehicle_name: str, topic_name: str, msg_type: str):
        super(GenericROSSubscriberComponent, self).__init__()
        self._vehicle_name: str = vehicle_name
        self._ros = ROS.get_connection(self._vehicle_name)
        topic_name: str = f"/{self._vehicle_name}/{topic_name.lstrip('/')}"
        self._topic: roslibpy.Topic = roslibpy.Topic(self._ros, topic_name, msg_type)
        # queues
        self._out_data: Queue[Any] = Queue()
        self._sleep: Queue[None] = Queue()

    @staticmethod
    @abstractmethod
    def _msg_to_data(msg) -> Any:
        pass

    def worker(self):
        if not self._ros.is_connected:
            self._ros.run()
        # subscribe to topic
        self._topic.subscribe(lambda msg: self._out_data.put(self._msg_to_data(msg)))
        # wait for the queue to shutdown (this is used to keep the thread alive)
        self._sleep.get()

    def stop(self):
        try:
            self._topic.unsubscribe()
        except:
            pass
        super(GenericROSSubscriberComponent, self).stop()


class CameraDriverComponent(GenericROSSubscriberComponent[BGRImage]):
    def __init__(self, vehicle_name: str):
        # TODO: you might want to expose the throttle to avoid swamping the websocket
        super(CameraDriverComponent, self).__init__(
            vehicle_name, "/camera_node/image/compressed", "sensor_msgs/CompressedImage"
        )
        # JPEG decoder
        self._jpeg_decoder = TurboJPEG()

    @property
    def out_bgr(self) -> Queue[BGRImage]:
        return self._out_data

    def _msg_to_data(self, msg) -> BGRImage:
        raw: bytes = msg['data'].encode('ascii')
        jpeg: JPEGImage = base64.b64decode(raw)
        return self._jpeg_decoder.decode(jpeg)


class TimeOfFlightDriverComponent(GenericROSSubscriberComponent[Range]):

    def __init__(self, vehicle_name: str):
        super(TimeOfFlightDriverComponent, self).__init__(
            vehicle_name, "/front_center_tof_driver_node/range", "sensor_msgs/Range"
        )

    @property
    def out_range(self) -> Queue[Range]:
        return self._out_data

    @staticmethod
    def _msg_to_data(msg) -> Optional[Range]:
        max_range: float = msg["max_range"]
        range: float = msg["range"]
        return None if range >= max_range else range
