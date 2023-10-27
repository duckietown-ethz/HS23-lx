from duckietown.types import BGRImage, Queue
from duckietown.components import Component

import cv2


class FlipImageComponent(Component[BGRImage, BGRImage]):
    """
    This is an example of a component that flips an image vertically.

    Args:
        axis: int       Axis along which the image is flipped. 0 = Vertical, 1 = Horizontal
    """

    def __init__(self, axis: int = 0):
        super(FlipImageComponent, self).__init__()
        self._axis: int = axis
        # queues
        self.in_bgr: Queue[BGRImage] = Queue()
        self.out_bgr: Queue[BGRImage] = Queue()

    def worker(self):
        bgr = self.in_bgr.get()
        # flip image
        bgr = cv2.flip(bgr, self._axis)
        # send out
        self.out_bgr.put(bgr)
