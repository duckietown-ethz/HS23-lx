import base64
import time
from abc import abstractmethod
from threading import Condition
from typing import Optional, Any, Tuple, List

from roslibpy import Message

# noinspection PyUnresolvedReferences
from turbojpeg import TurboJPEG

from .base import Component, OutputType, InputType
from ..ros import ROS, RosTopic
from ..types import JPEGImage, BGRImage, IQueue, Queue, CallbackQueue, PWMSignal, LEDsPattern, RGBAColor, \
    Range, Ticks

__all__ = [
    "ROSCameraDriverComponent",
    "ROSTimeOfFlightDriverComponent",
    "ROSWheelEncoderDriverComponent",
    "ROSMotorsDriverComponent",
    "ROSLEDsDriverComponent",
    "GenericROSPublisherComponent",
    "GenericROSSubscriberComponent"
]


class GenericROSSubscriberComponent(Component[None, OutputType]):

    def __init__(self, vehicle_name: str, topic_name: str, msg_type: str,
                 compression=None,
                 latch: bool = False,
                 frequency: float = 0,
                 queue_size: int = 1,
                 queue_length: int = 1,
                 reconnect_on_close: bool = True):
        super(GenericROSSubscriberComponent, self).__init__()
        self._vehicle_name: str = vehicle_name
        self._ros = ROS.get_connection(self._vehicle_name)
        topic_name: str = f"/{self._vehicle_name}/{topic_name.lstrip('/')}"
        # convert frequency to throttle rate (ms between messages)
        throttle_rate: int = int(1000 * (1.0 / frequency)) if frequency > 0 else 0
        # create underlying topic
        self._topic: RosTopic = RosTopic(
            self._ros,
            topic_name,
            msg_type,
            compression=compression,
            latch=latch,
            throttle_rate=throttle_rate,
            queue_size=queue_size,
            queue_length=queue_length,
            reconnect_on_close=reconnect_on_close
        )
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
        # wait for the queue to shut down (this is used to keep the thread alive)
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
        self._topic: RosTopic = RosTopic(self._ros, topic_name, msg_type)
        # data override
        self._override_msg: Optional[dict] = None
        # simulate Thread.join
        self._join: Condition = Condition()
        # queues
        self._in_data: IQueue[Any] = CallbackQueue(self._publish)

    @staticmethod
    @abstractmethod
    def _data_to_msg(data) -> dict:
        pass

    def start(self) -> None:
        # let the parent class start
        super(GenericROSPublisherComponent, self).start()
        # connect to ROS
        if not self._ros.is_connected:
            self._ros.run()

    def join(self, **kwargs) -> None:
        if self.is_shutdown:
            return
        with self._join:
            self._join.wait()

    def worker(self):
        pass

    def _publish(self, data: Any, *, force: bool = False):
        if not self.is_started and not force:
            return
        if self.is_shutdown and not force:
            return
        # (re)advertise the topic if necessary
        if not self._topic.is_advertised and not self.is_shutdown:
            self._topic.advertise()
        # format message
        msg: dict = self._data_to_msg(data) if not self._override_msg else self._override_msg
        # publish message
        self._topic.publish(Message(msg))

    def reset(self):
        super().reset()
        self._override_msg = None

    def stop(self):
        # let the parent class start the cleaning process (important that we do this first)
        super(GenericROSPublisherComponent, self).stop()
        # release all Threads that have joined this object
        with self._join:
            self._join.notify_all()
        # unadvertise the topic
        try:
            self._topic.unadvertise()
        except:
            pass


class ROSCameraDriverComponent(GenericROSSubscriberComponent[BGRImage]):
    def __init__(self, vehicle_name: str, **kwargs):
        super(ROSCameraDriverComponent, self).__init__(
            vehicle_name, "/camera_node/image/compressed", "sensor_msgs/CompressedImage", **kwargs
        )
        self._compression: Optional[str] = kwargs.get("compression", None)
        # JPEG decoder
        self._jpeg_decoder = TurboJPEG()

    @property
    def out_bgr(self) -> IQueue[BGRImage]:
        return self._out_data

    def _msg_to_data(self, msg) -> BGRImage:
        if self._topic.compression == "cbor":
            jpeg: JPEGImage = msg['data']
        elif self._topic.compression == "none":
            raw: bytes = msg['data'].encode('ascii')
            jpeg: JPEGImage = base64.b64decode(raw)
        else:
            raise ValueError(f"Compression '{self._compression}' not supported")
        # ---
        return self._jpeg_decoder.decode(jpeg)


class ROSTimeOfFlightDriverComponent(GenericROSSubscriberComponent[Range]):

    def __init__(self, vehicle_name: str, **kwargs):
        super(ROSTimeOfFlightDriverComponent, self).__init__(
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


class ROSWheelEncoderDriverComponent(GenericROSSubscriberComponent[Range]):

    def __init__(self, vehicle_name: str, side: str, **kwargs):
        if side not in ["left", "right"]:
            raise ValueError(f"Side '{side}' not recognized. Valid choices are ['left', 'right'].")
        super(ROSWheelEncoderDriverComponent, self).__init__(
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


class ROSLEDsDriverComponent(GenericROSPublisherComponent[LEDsPattern]):

    OFF: RGBAColor = (0, 0, 0, 0)
    IDLE: LEDsPattern = LEDsPattern(
        # white on the front
        front_left=(1, 1, 1, 0.1),
        front_right=(1, 1, 1, 0.1),
        # red on the back
        rear_right=(1, 0, 0, 0.2),
        rear_left=(1, 0, 0, 0.2),
    )

    def __init__(self, vehicle_name: str):
        super(ROSLEDsDriverComponent, self).__init__(
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

    def stop(self):
        super(ROSLEDsDriverComponent, self).stop()
        self._publish(self.IDLE, force=True)


class ROSMotorsDriverComponent(GenericROSPublisherComponent[Tuple[PWMSignal, PWMSignal]]):

    OFF: float = 0.0

    def __init__(self, vehicle_name: str):
        super(ROSMotorsDriverComponent, self).__init__(
            vehicle_name, "/wheels_driver_node/wheels_cmd", "duckietown_msgs/WheelsCmdStamped"
        )
        # queues
        self.out_command_time: Queue[float] = Queue()

    @property
    def in_pwml_pwmr(self) -> IQueue[Tuple[PWMSignal, PWMSignal]]:
        return self._in_data

    def _data_to_msg(self, data: Tuple[PWMSignal, PWMSignal]) -> dict:
        return {
            "vel_left": data[0],
            "vel_right": data[1],
        }

    def _publish(self, data: Any, *, force: bool = False):
        super(ROSMotorsDriverComponent, self)._publish(data, force=force)
        self.out_command_time.put(time.time())

    def stop(self):
        self._override_msg = self._data_to_msg((self.OFF, self.OFF))
        # send the 0, 0 command multiple times
        for _ in range(5):
            self._publish((self.OFF, self.OFF), force=True)
            time.sleep(1. / 60.)
        # ---
        super(ROSMotorsDriverComponent, self).stop()
