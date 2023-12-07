from typing import Dict, Any

from .ros import GenericROSSubscriberComponent, GenericROSPublisherComponent
from ..types import IQueue


class RawSubscriberComponent(GenericROSSubscriberComponent[Dict[str, Any]]):
    def __init__(self, vehicle_name: str, topic: str, message_type: str, **kwargs):
        assert topic.startswith("/")
        topic = topic[len(f"/{vehicle_name}"):] if topic.startswith(f"/{vehicle_name}") else topic
        # TODO: you might want to expose the throttle to avoid swamping the websocket
        super(RawSubscriberComponent, self).__init__(vehicle_name, topic, message_type, **kwargs)

    @property
    def out_raw(self) -> IQueue[Dict[str, Any]]:
        return self._out_data

    def _msg_to_data(self, msg) -> Dict[str, Any]:
        return msg


class RawPublisherComponent(GenericROSPublisherComponent[Dict[str, Any]]):
    def __init__(self, vehicle_name: str, topic: str, message_type: str):
        assert topic.startswith("/")
        topic = topic[len(f"/{vehicle_name}"):] if topic.startswith(f"/{vehicle_name}") else topic
        # TODO: you might want to expose the throttle to avoid swamping the websocket
        super(RawPublisherComponent, self).__init__(vehicle_name, topic, message_type)

    @property
    def in_raw(self) -> IQueue[Dict[str, Any]]:
        return self._in_data

    @staticmethod
    def _data_to_msg(data: Dict[str, Any]) -> dict:
        assert isinstance(data, dict)
        return data
