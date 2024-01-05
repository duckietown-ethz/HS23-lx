from abc import ABC
from typing import Optional, Union, Any, Callable

import cv2
from IPython.core.display import DisplayObject
from IPython.display import display, Image, Markdown

from dt_class_utils import DTReminder

from .base import Component, InputType
from ..types import JPEGImage, BGRImage, Queue

__all__ = [
    "ImageRendererComponent",
    "TextRendererComponent",
    "MarkdownRendererComponent"
]


class GenericRenderingComponent(Component[InputType, None], ABC):

    def __init__(self, initial: DisplayObject, disp: Optional[display] = None):
        super(GenericRenderingComponent, self).__init__()
        self._display: display = disp or display(initial, display_id=True)

    def join(self):
        try:
            super(GenericRenderingComponent, self).join()
        except KeyboardInterrupt:
            pass


class ImageRendererComponent(GenericRenderingComponent[Union[JPEGImage, BGRImage]]):

    def __init__(self, disp: Optional[display] = None, frequency: float = None, image_opts: dict = None):
        self._image_opts: dict = image_opts or {}
        self._image: Image = Image(data=b"", **self._image_opts)
        super(ImageRendererComponent, self).__init__(self._image, disp)
        self._frequency: float = frequency or 0
        self._reminder: DTReminder = DTReminder(frequency=self._frequency)
        # queues
        self.in_image: Queue[Union[JPEGImage, BGRImage]] = Queue()

    def worker(self):
        # consume frames
        while not self.is_shutdown:
            frame: Union[JPEGImage, BGRImage] = self.in_image.get()
            jpeg: JPEGImage

            if self._reminder.is_time():
                # JPEG
                if isinstance(frame, JPEGImage):
                    jpeg = frame
                # BGR
                else:
                    # bgr -> jpeg
                    _, frame = cv2.imencode('.jpeg', frame)
                    jpeg = frame.tobytes()

                # render frame
                # self._display.update(Image(data=jpeg, **self._image_opts, retina=True))
                self._image.data = jpeg
                self._display.update(self._image)


class TextRendererComponent(GenericRenderingComponent[str]):

    def __init__(self, disp: Optional[display] = None):
        super(TextRendererComponent, self).__init__(Markdown(data=""), disp)
        # queues
        self.in_data: Queue[Any] = Queue()

    def worker(self):
        # consume inputs
        while not self.is_shutdown:
            data: Any = self.in_data.get()
            # render frame
            self._display.update(Markdown(data=str(data)))


class MarkdownRendererComponent(GenericRenderingComponent[str]):

    def __init__(self, formatter: Callable[[Any], str] = str, disp: Optional[display] = None):
        super(MarkdownRendererComponent, self).__init__(Markdown(data=""), disp)
        self.formatter: Callable[[Any], str] = formatter
        # queues
        self.in_data: Queue[Any] = Queue()

    def worker(self):
        # consume inputs
        while not self.is_shutdown:
            data: Any = self.in_data.get()
            # format input data
            markdown: str = self.formatter(data)
            # render frame
            self._display.update(Markdown(data=markdown))
