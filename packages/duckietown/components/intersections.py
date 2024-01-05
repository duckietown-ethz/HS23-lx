import time
from typing import Dict, List, Tuple

import numpy as np

from dt_computer_vision.ground_projection import GroundPoint
from dt_state_estimation.lane_filter.types import Segment, SegmentColor

from duckietown.types import Queue, LEDsPattern

from .base import Component
from ..types import Queue, DetectedLines, ColorName


class StopLineDetectorComponent(Component[Dict[ColorName, DetectedLines], bool]):
    """
    The Stop Line Detector can be used to detect when a Duckiebot is facing a red stop-line.

    Args:
        TODO

    """

    def __init__(self, stop_distance: float = 0.25, min_segs: float = 12, max_y: float = 0.2):
        super(StopLineDetectorComponent, self).__init__()
        # store arguments
        self._stop_distance = stop_distance
        self._min_segs = min_segs
        self._max_y = max_y
        # queues
        self.in_d_phi: Queue[Tuple[D, Phi]] = Queue(repeat_last=True)
        self.in_segments: Queue[List[Segment]] = Queue()
        self.out_stopline: Queue[bool] = Queue()

    def worker(self):
        while not self.is_shutdown:
            segments: List[Segment] = self.in_segments.get()

            good_seg_count: int = 0
            stop_line_x_accumulator: int = 0.0
            stop_line_y_accumulator: int = 0.0

            for segment in segments:
                # filter by color
                if segment.color != SegmentColor.RED:
                    continue

                # pixels
                p0: GroundPoint = segment.points[0]
                p1: GroundPoint = segment.points[1]

                if p0.x < 0 or p1.x < 0:  # the point is behind us
                    continue

                p1_lane = self.to_lane_frame(p0)
                p2_lane = self.to_lane_frame(p1)

                avg_x = 0.5 * (p1_lane[0] + p2_lane[0])
                avg_y = 0.5 * (p1_lane[1] + p2_lane[1])
                stop_line_x_accumulator += avg_x
                stop_line_y_accumulator += avg_y  # TODO output covariance and not just mean
                good_seg_count += 1.0


            if good_seg_count < self._min_segs:
                stop_line_detected = False
                at_stop_line = False

            else:
                stop_line_detected = True
                
                stop_line_point_x = stop_line_x_accumulator / good_seg_count
                stop_line_point_y = stop_line_y_accumulator / good_seg_count
                
                at_stop_line = stop_line_point_x < self._stop_distance and np.abs(stop_line_point_y) < self._max_y
                
            # self.out_stopline.put((stop_line_point_x, self._stop_distance, np.abs(stop_line_point_y), self._max_y, at_stop_line))
            # print(int(good_seg_count))

            self.out_stopline.put(at_stop_line)

    def to_lane_frame(self, point):
        d, phi = self.in_d_phi.get()
        p_homo = np.array([point.x, point.y, 1])
        T = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), d], [0, 0, 1]])
        p_new_homo = T.dot(p_homo)
        p_new = p_new_homo[0:2]
        return p_new


class StopLineLEDComponent(Component[bool, LEDsPattern]):

    PATTERNS: Dict[bool, LEDsPattern] = {
        False: LEDsPattern(
            # low intensity on the front and back
            front_left=(1, 1, 1, 0.6),
            front_right=(1, 1, 1, 0.6),
            rear_right=(1, 0, 0, 0.2),
            rear_left=(1, 0, 0, 0.2),
        ),
        True: LEDsPattern(
            # low intensity on the front, high intensity in the back (breaking)
            front_left=(1, 1, 1, 0.6),
            front_right=(1, 1, 1, 0.6),
            rear_right=(1, 0, 0, 1.0),
            rear_left=(1, 0, 0, 1.0),
        )
    }

    def __init__(self):
        super(StopLineLEDComponent, self).__init__()
        # queues
        self.in_break: Queue[bool] = Queue(repeat_last=True, initial=False)
        self.out_pattern: Queue[LEDsPattern] = Queue()
    
    def worker(self):
        while not self.is_shutdown:
            breaking: bool = self.in_break.get()
            # get color
            pattern: LEDsPattern = self.PATTERNS[breaking]
            # send pattern out
            self.out_pattern.put(pattern)
            # ---
            time.sleep(0.2)
