import base64
import dataclasses
from abc import abstractmethod
from typing import Optional, Any, Tuple, List

import roslibpy
from turbojpeg import TurboJPEG

from .base import Component, OutputType, InputType
from ..ros import ROS
from ..types import JPEGImage, BGRImage, IQueue, Queue, CallbackQueue

Range = float
Ticks = int
RGBAColor = Tuple[float, float, float, float]

__all__ = [
    "CameraDriverComponent",
    "TimeOfFlightDriverComponent",
    "WheelEncoderDriverComponent"
]


class GenericROSSubscriberComponent(Component[None, OutputType]):

    def __init__(self, vehicle_name: str, topic_name: str, msg_type: str, throttle_rate: int = 0):
        super(GenericROSSubscriberComponent, self).__init__()
        self._vehicle_name: str = vehicle_name
        self._ros = ROS.get_connection(self._vehicle_name)
        topic_name: str = f"/{self._vehicle_name}/{topic_name.lstrip('/')}"
        self._topic: roslibpy.Topic = roslibpy.Topic(self._ros, topic_name, msg_type,
                                                     throttle_rate=throttle_rate)
        # queues
        self._out_data: IQueue[Any] = Queue()
        self._sleep: IQueue[None] = Queue()

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


class GenericROSPublisherComponent(Component[InputType, None]):

    def __init__(self, vehicle_name: str, topic_name: str, msg_type: str):
        super(GenericROSPublisherComponent, self).__init__()
        self._vehicle_name: str = vehicle_name
        self._ros = ROS.get_connection(self._vehicle_name)
        topic_name: str = f"/{self._vehicle_name}/{topic_name.lstrip('/')}"
        self._topic: roslibpy.Topic = roslibpy.Topic(self._ros, topic_name, msg_type)
        # queues
        self._in_data: IQueue[Any] = CallbackQueue(self._publish)

    @staticmethod
    @abstractmethod
    def _data_to_msg(data) -> dict:
        pass

    def start(self) -> None:
        if not self._ros.is_connected:
            self._ros.run()

    def join(self, **kwargs) -> None:
        raise RuntimeError("You cannot join a ROS publisher object.")

    def worker(self):
        pass

    def _publish(self, data: Any):
        # format message
        msg: dict = self._data_to_msg(data)
        # publish message
        self._topic.publish(roslibpy.Message(msg))

    def stop(self):
        try:
            self._topic.unadvertise()
        except:
            pass
        super(GenericROSPublisherComponent, self).stop()


class CameraDriverComponent(GenericROSSubscriberComponent[BGRImage]):
    def __init__(self, vehicle_name: str, **kwargs):
        # TODO: you might want to expose the throttle to avoid swamping the websocket
        super(CameraDriverComponent, self).__init__(
            vehicle_name, "/camera_node/image/compressed", "sensor_msgs/CompressedImage", **kwargs
        )
        # JPEG decoder
        self._jpeg_decoder = TurboJPEG()

    @property
    def out_bgr(self) -> IQueue[BGRImage]:
        return self._out_data

    def _msg_to_data(self, msg) -> BGRImage:
        raw: bytes = msg['data'].encode('ascii')
        jpeg: JPEGImage = base64.b64decode(raw)
        return self._jpeg_decoder.decode(jpeg)


class TimeOfFlightDriverComponent(GenericROSSubscriberComponent[Range]):

    def __init__(self, vehicle_name: str, **kwargs):
        super(TimeOfFlightDriverComponent, self).__init__(
            vehicle_name, "/front_center_tof_driver_node/range", "sensor_msgs/Range", **kwargs
        )

    @property
    def out_range(self) -> IQueue[Range]:
        return self._out_data

    @staticmethod
    def _msg_to_data(msg) -> Optional[Range]:
        max_range: float = msg["max_range"]
        range: float = msg["range"]
        return None if range >= max_range else range


class WheelEncoderDriverComponent(GenericROSSubscriberComponent[Range]):

    def __init__(self, vehicle_name: str, side: str, **kwargs):
        if side not in ["left", "right"]:
            raise ValueError(f"Side '{side}' not recognized. Valid choices are ['left', 'right'].")
        super(WheelEncoderDriverComponent, self).__init__(
            vehicle_name, f"/{side}_wheel_encoder_node/tick", "duckietown_msgs/WheelEncoderStamped", **kwargs
        )
        self._resolution: Optional[float] = None
        # queues
        self.out_rotation: IQueue[float] = Queue()

    @property
    def out_ticks(self) -> IQueue[Ticks]:
        return self._out_data

    @property
    def resolution(self) -> Optional[float]:
        return self._resolution

    def _msg_to_data(self, msg) -> Optional[Range]:
        self._resolution = msg["resolution"]
        ticks: int = msg["data"]
        rotation: float = ticks / self._resolution
        self.out_rotation.put(rotation)
        return ticks


@dataclasses.dataclass
class LEDsPattern:
    front_left: RGBAColor
    front_right: RGBAColor
    rear_right: RGBAColor
    rear_left: RGBAColor


class LEDsDriverComponent(GenericROSPublisherComponent[LEDsPattern]):

    OFF: RGBAColor = (0, 0, 0, 0)

    def __init__(self, vehicle_name: str):
        super(LEDsDriverComponent, self).__init__(
            vehicle_name, "/led_emitter_node/led_pattern", "duckietown_msgs/LEDPattern"
        )

    @property
    def in_pattern(self) -> IQueue[LEDsPattern]:
        return self._in_data

    def _data_to_msg(self, data: LEDsPattern) -> dict:
        leds: List[RGBAColor] = [data.front_left, data.rear_left, self.OFF, data.rear_right, data.front_right]
        return {
            "rgb_vals": [
                dict(zip("rgba", led)) for led in leds
            ]
        }
