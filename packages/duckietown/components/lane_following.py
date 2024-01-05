import time
from typing import Dict, List, Tuple

import numpy as np

from dt_computer_vision.camera import CameraModel, Pixel, NormalizedImagePoint
from dt_computer_vision.ground_projection import GroundProjector, GroundPoint
from dt_computer_vision.ground_projection.rendering import draw_grid_image, debug_image
from dt_computer_vision.line_detection import ColorRange, Detections, LineDetector
from dt_computer_vision.line_detection.rendering import draw_segments
from dt_modeling.electronics.PWM import PWM
from dt_modeling.kinematics.inverse import InverseKinematics
from dt_motion_planning.lane_controller import PIDLaneController
from dt_state_estimation.lane_filter import LaneFilterHistogram
from dt_state_estimation.lane_filter.rendering import plot_d_phi
from dt_state_estimation.lane_filter.types import Segment, SegmentColor

from .base import Component
from ..types import CameraParameters, BGRImage, RGB8Color, Queue, DetectedLines, ColorName

V, Omega = float, float
D, Phi = float, float
OmegaLeft, OmegaRight = float, float
PWMLeft, PWMRight = float, float

# colors
# DEFAULT_COLOR_RANGES: Dict[str, ColorRange] = {
#     "white": ColorRange.fromDict({"low": [10, 0, 120], "high": [220, 100, 220]}),
#     "yellow": ColorRange.fromDict({"low": [0, 100, 100], "high": [40, 255, 220]}),
#     # "red": ColorRange.fromDict({
#     #     "low_1": [0, 140, 100],
#     #     "high_1": [15, 255, 255],
#     #     "low_2": [165, 140, 100],
#     #     "high_2": [180, 255, 255],
#     # }),
# }


DEFAULT_COLOR_RANGES: Dict[str, ColorRange] = {
    "white": ColorRange.fromDict({"low": [0, 0, 180], "high": [220, 220, 220]}),
    "yellow": ColorRange.fromDict({"low": [0, 40, 100], "high": [40, 255, 255]}),
    "red": ColorRange.fromDict({
        "low": [165, 140, 100],
        "high": [180, 255, 255],
    }),
}




# GOOD ranges - Belmont Media Center - Demo
DEFAULT_COLOR_RANGES: Dict[str, ColorRange] = {
    "white": ColorRange.fromDict({"low": [0, 0, 180], "high": [220, 220, 220]}),
    "yellow": ColorRange.fromDict({"low": [0, 40, 100], "high": [40, 255, 255]}),
}



COLORS: Dict[str, RGB8Color] = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "white": (255, 255, 255),
}

# distance between wheels (meters)
WHEEL_BASELINE: float = 0.1

# wheel radius (meters)
WHEEL_RADIUS: float = 0.0318

__all__ = [
    "ImageCropComponent",
    "LineDetectorComponent",
    "LaneFilterComponent",
    "LaneControllerComponent",
    "InverseKinematicsComponent",
    "PWMComponent",
]


class ImageCropComponent(Component[BGRImage, BGRImage]):
    """
    Represents a virtual camera that takes in a frame, the camera parameters of the original camera
    (i.e., the camera that produced the frame), and the number of pixels to crop from the top. It then outputs
    the corresponding cropped image as a BGR numpy array together with the camera parameters for the
    "cropped" camera.

    Args:
        parameters: CameraParameters     The parameters of the original camera in the format used by ROS.
                           Docs: https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html

       crop_top: int                    Number of pixels to crop from the top of the image. Default: 240px
    """

    # default values
    DEFAULT_CAMERA_PARAMETERS: CameraParameters = {
        "width": 640,
        "height": 480,
        # "K": np.reshape(
        #     [
        #         295.79606866959824,
        #         0.0,
        #         321.2621599038631,
        #         0.0,
        #         299.5389048862878,
        #         241.73616515312332,
        #         0.0,
        #         0.0,
        #         1.0,
        #     ],
        #     (3, 3),
        # ),
        # "D": [
        #     -0.23543978771661125,
        #     0.03637781479419574,
        #     -0.0033069818601306755,
        #     -0.0012140708179525926,
        #     0.0,
        # ],
        # "P": np.reshape(
        #     [
        #         201.14027404785156,
        #         0.0,
        #         319.5586620845679,
        #         0.0,
        #         0.0,
        #         239.74398803710938,
        #         237.60151004037834,
        #         0.0,
        #         0.0,
        #         0.0,
        #         1.0,
        #         0.0,
        #     ],
        #     (3, 4),
        # ),
        # "H": np.reshape(
        #     [
        #         8.56148231e-03,
        #         2.22480148e-01,
        #         4.24318934e-01,
        #         -5.67022044e-01,
        #         -1.13258040e-03,
        #         6.81113839e-04,
        #         5.80917161e-02,
        #         4.35079347e00,
        #         1.0,
        #     ],
        #     (3, 3),
        # ),
    }

    def __init__(self, parameters: CameraParameters, crop_top: int = 240):
        super(ImageCropComponent, self).__init__()
        self.parameters: CameraParameters = parameters
        self._crop_top: int = crop_top
        # compute image crop
        self._image_crop = [0, crop_top, self.parameters["width"], self.parameters["height"] - crop_top]
        # compute new cropped camera parameters
        x, y, w, h = self._image_crop
        # - update K
        _K = self.parameters["K"]
        _K[0][2] = _K[0][2] - x
        _K[1][2] = _K[1][2] - y
        # - update P
        _P = self.parameters["P"]
        _P[0][2] = _P[0][2] - x
        _P[1][2] = _P[1][2] - y
        # queues
        self.in_bgr: Queue[BGRImage] = Queue()
        self.out_bgr: Queue[BGRImage] = Queue()

    def worker(self):
        while not self.is_shutdown:
            bgr: BGRImage = self.in_bgr.get()
            # crop frame
            x, y, w, h = self._image_crop
            bgr = bgr[y : y + h, x : x + w, :]
            # send out
            self.out_bgr.put(bgr)


class LineDetectorComponent(Component[BGRImage, Dict[ColorName, DetectedLines]]):
    """
    The Line Detector can be used to extract line segments from a particular color range in
    an image. It combines edge detection, color filtering, and line segment extraction.

    This class was created for the goal of extracting the white, yellow, and red lines in the
    Duckiebot's camera stream as part of the lane localization pipeline. It is setup in a way
    that allows efficient detection of line segments in different color ranges.

    In order to process an image, first the :py:meth:`setImage` method must be called.
    In makes an internal copy of the image, converts it to `HSV color space
    <https://en.wikipedia.org/wiki/HSL_and_HSV>`_, which is much better for color segmentation,
    and applies `Canny edge detection <https://en.wikipedia.org/wiki/Canny_edge_detector>`_.

    Then, to do the actual line segment extraction, a call to :py:meth:`detect` with a
    :py:class:`ColorRange` object must be made.
    Multiple such calls with different colour ranges can be made and these will reuse the
    precomputed HSV image and Canny edges.

    CV2 Documentation: https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html

    Args:
        color_ranges: Dict[str, ColorRange] dictionary of color ranges for each color of interest

        canny_thresholds: List[int]         a list with two entries that specify the thresholds for the
                                            hysteresis procedure
                                                Default: [80, 200]
        canny_aperture_size: int            aperture size for a Sobel operator
                                                Default: 3
        dilation_kernel_size: int           kernel size for the dilation operation which fills in the
                                            gaps in the color filter result
                                                Default: 3
        hough_threshold: int                accumulator threshold parameter. Only those lines are
                                            returned that get enough votes
                                                Default: 2
        hough_min_line_length: int          minimum line length. Line segments shorter than that are
                                            rejected
                                                Default: 3
        hough_max_line_gap: int             maximum allowed gap between points on the same line to
                                                link them
                                                    Default: 1

    """

    def __init__(self, color_ranges: Dict[str, ColorRange] = None, **kwargs):
        super(LineDetectorComponent, self).__init__()
        self._color_ranges: Dict[str, ColorRange] = color_ranges or DEFAULT_COLOR_RANGES
        # create line detector
        self._color_order = ["white", "yellow"]# , "red"]
        self._colors_to_detect = [self._color_ranges[c] for c in self._color_order]
        self._detector: LineDetector = LineDetector(**kwargs)
        # queues
        self.in_bgr: Queue[BGRImage] = Queue()
        self.out_lines: Queue[Dict[ColorName, DetectedLines]] = Queue()
        # debug queues
        self.out_lines_image: Queue[BGRImage] = Queue()

    def worker(self):
        while not self.is_shutdown:
            bgr: BGRImage = self.in_bgr.get()
            color_detections: List[Detections] = self._detector.detect(bgr, self._colors_to_detect)
            lines: Dict[str, dict] = {}
            for i, detections in enumerate(color_detections):
                color = self._color_order[i]
                # pack detections in a dictionary
                lines[color] = {
                    "lines": detections.lines.tolist(),
                    "centers": detections.centers.tolist(),
                    "normals": detections.normals.tolist(),
                    "color": self._color_ranges[color].representative,
                }
            self.out_lines.put(lines)

            # draw detections on top of the image
            segments_to_render: Dict[ColorRange, Detections] = {
                self._color_ranges[c]: color_detections[self._color_order.index(c)]
                for c in self._color_order
            }
            image_w_dets = draw_segments(bgr, segments_to_render)
            self.out_lines_image.put(image_w_dets)


class LaneFilterComponent(Component[Dict[ColorName, DetectedLines], Tuple[D, Phi]]):
    """
    Generates an estimate of the lane pose.

    Creates and maintain a histogram grid filter to estimate the lane pose.

    Lane pose is defined as the tuple (`d`, `phi`) : lateral deviation and angulare deviation
    from the center of the lane.

    - Predict step : Uses the estimated linear and angular velocities to predict the change in
    the lane pose.

    - Update Step : The filter receives a segment list. For each segment, it extracts the
    corresponding lane pose "votes", and adds it to the corresponding part of the histogram.

    Best estimate correspond to the slot of the histogram with the highest voted value.

    Args:
        camera_parameters: dict
        mean_d_0: float                     Default: 0
        mean_phi_0: float                   Default: 0
        sigma_d_0: float                    Default: 0.1
        sigma_phi_0: float                  Default: 0.1
        delta_d: float                      Default: 0.02
        delta_phi: float                    Default: 5 degrees
        d_max: float                        Default: 0.3
        d_min: float                        Default: -0.15
        phi_min: float                      Default: -85 degrees
        phi_max: float                      Default: 85 degrees
        cov_v: float                        Default: 0.5
        linewidth_white: float              Default: 0.05
        linewidth_yellow: float             Default: 0.025
        lanewidth: float                    Default: 0.23
        min_max: float                      Default: 0.1
        sigma_d_mask: float                 Default: 1.0
        sigma_phi_mask: float               Default: 2.0
        curvature_res: float                Default: 0
        range_min: float                    Default: 0.2
        range_est: float                    Default: 0.33
        range_max: float                    Default: 0.6
        curvature_right: float              Default: -0.054
        curvature_left: float               Default: 0.025

    """

    def __init__(self, camera_parameters: CameraParameters, **kwargs):
        super(LaneFilterComponent, self).__init__()
        self._last_prediction_time: float = 0.0
        # create new camera model
        self._camera = CameraModel(
            width=camera_parameters["width"],
            height=camera_parameters["height"],
            K=camera_parameters["K"],
            D=camera_parameters["D"],
            P=camera_parameters["P"],
            H=camera_parameters["H"],
        )
        self._projector: GroundProjector = GroundProjector(self._camera)
        # create filter
        self._filter: LaneFilterHistogram = LaneFilterHistogram(**kwargs, linewidth_yellow=0.05)
        self._grid: BGRImage = draw_grid_image((400, 400))
        # queues
        self.in_lines: Queue[Dict[ColorName, DetectedLines]] = Queue()
        self.in_v_omega: Queue[Tuple[V, Omega]] = Queue(repeat_last=True)
        self.in_command_time: Queue[float] = Queue(repeat_last=True, initial=0.0)
        self.out_d_phi: Queue[Tuple[D, Phi]] = Queue()
        self.out_segments: Queue[List[Segment]] = Queue()
        self.out_segments_image: Queue[BGRImage] = Queue()
        self.out_belief_image: Queue[BGRImage] = Queue()

    def worker(self):
        while not self.is_shutdown:
            segments: List[Segment] = []
            colored_segments: Dict[RGB8Color, List[Tuple[GroundPoint, GroundPoint]]] = {}
            lines: Dict[ColorName, DetectedLines] = self.in_lines.get()

            for color, colored_lines in lines.items():
                grounded_segments: List[Tuple[GroundPoint, GroundPoint]] = []
                for line in colored_lines["lines"]:
                    # distorted pixels
                    p0: Pixel = Pixel(line[0], line[1])
                    p1: Pixel = Pixel(line[2], line[3])
                    # distorted pixels to rectified pixels
                    p0_rect: Pixel = self._camera.rectifier.rectify_pixel(p0)
                    p1_rect: Pixel = self._camera.rectifier.rectify_pixel(p1)
                    # rectified pixel to normalized coordinates
                    p0_norm: NormalizedImagePoint = self._camera.pixel2vector(p0_rect)
                    p1_norm: NormalizedImagePoint = self._camera.pixel2vector(p1_rect)
                    # project image point onto the ground plane
                    grounded_p0: GroundPoint = self._projector.vector2ground(p0_norm)
                    grounded_p1: GroundPoint = self._projector.vector2ground(p1_norm)
                    # add grounded segment to output
                    # noinspection PyTypeChecker
                    segments.append(Segment(points=[grounded_p0, grounded_p1], color=SegmentColor(color)))
                    grounded_segments.append((grounded_p0, grounded_p1))

                # add segments to list of segments for this color
                colored_segments[COLORS[color]] = grounded_segments

            # publish ground segments
            self.out_segments.put(segments)

            # draw segments
            if self.out_segments_image.anybody_interested:
                image_w_segs: BGRImage = debug_image(
                    colored_segments, (400, 400), background_image=self._grid
                )
                self.out_segments_image.put(image_w_segs)

            # apply update
            self._filter.update(segments)

            # perform prediction step
            last_command_time: float = self.in_command_time.get()
            if last_command_time > self._last_prediction_time:
                now: float = time.time()
                delta_t: float = now - self._last_prediction_time
                last_command: Tuple[V, Omega] = self.in_v_omega.get()
                self._filter.predict(delta_t, *last_command)
                self._last_prediction_time = now

            # perform estimation step
            d_hat, phi_hat = self._filter.get_estimate()

            # print belief
            if self.out_belief_image.anybody_interested:
                belief: BGRImage = plot_d_phi(d=d_hat, phi=phi_hat)
                self.out_belief_image.put(belief)

            # publish d, phi
            self.out_d_phi.put((d_hat, phi_hat))


class LaneControllerComponent(Component[Tuple[D, Phi], Tuple[V, Omega]]):
    """
    The Lane Controller component can be used to compute control commands from pose estimations.

    The control commands are in terms of linear and angular velocity (v, omega).
    The inputs are errors in the relative pose of the Duckiebot in the current lane (d, phi).

    This implementation is a simple PI(D) controller.

    Args:
        v_bar (:obj:`float`): Nominal velocity in m/s
        k_d (:obj:`float`): Proportional term for lateral deviation
        k_theta (:obj:`float`): Proportional term for heading deviation
        k_Id (:obj:`float`): integral term for lateral deviation
        k_Iphi (:obj:`float`): integral term for lateral deviation
        d_thres (:obj:`float`): Maximum value for lateral error
        phi_thres (:obj:`float`): Maximum value for heading error
        d_offset (:obj:`float`): Goal offset from center of the lane
        phi_offset (:obj:`float`): Goal offset from heading parallel to center of the lane
        d_resolution (:obj:`float`): Resolution of lateral position estimate
        phi_resolution (:obj:`float`): Resolution of heading estimate
        omega_ff (:obj:`float`): Feedforward part of controller
        integral_bounds (:obj:`dict`): Bounds for integral term
        stop_slowdown (:obj:`dict`): Start and end distances for slowdown at stops

    """

    def __init__(self, **kwargs):
        super(LaneControllerComponent, self).__init__()
        # create PID
        self._pid = PIDLaneController(**kwargs)
        # queues
        self.in_d_phi: Queue[Tuple[D, Phi]] = Queue()
        self.out_v_omega: Queue[Tuple[V, Omega]] = Queue()

    def worker(self):
        while not self.is_shutdown:
            d_hat, phi_hat = self.in_d_phi.get()
            self._pid.update(d_hat, phi_hat, time.time())
            v, w = self._pid.compute_commands()
            self.out_v_omega.put((v, w))


class InverseKinematicsComponent(Component[Tuple[V, Omega], Tuple[OmegaLeft, OmegaRight]]):
    """
    The `InverseKinematics` maps car speeds at the chassis level to wheel commands that the robot
    should execute.
    It utilises the car geometry to calculate the wheels' rotation needed for the robot to perform
    the desired chassis commands.

    Args:
        v_max (:obj:`float`):           limits the input velocity
        omega_max (:obj:`float`):       limits the input steering angle

    """

    def __init__(self, v_max: float = 1.0, omega_max: float = 5.0):
        super(InverseKinematicsComponent, self).__init__()
        # create IK mapper
        self._ik = InverseKinematics(
            wheel_baseline=WHEEL_BASELINE, wheel_radius=WHEEL_RADIUS, v_max=v_max, omega_max=omega_max
        )
        # queues
        self.in_v_omega: Queue[Tuple[V, Omega]] = Queue()
        self.out_wl_wr: Queue[Tuple[OmegaLeft, OmegaRight]] = Queue()

    def worker(self):
        while not self.is_shutdown:
            v, w = self.in_v_omega.get()
            wl, wr = self._ik.get_wheels_speed(v, w)
            self.out_wl_wr.put((wl, wr))


class PWMComponent(Component[Tuple[OmegaLeft, OmegaRight], Tuple[PWMLeft, PWMRight]]):
    """
    The `PWM` maps wheels' speed to duty cycle commands to send to the DC motors.
    It calculates the PWM commands that the DC motors should execute in order for the robot to
    perform a desired wheels' command.

    Args:
        gain: float       scaling factor applied to the desired velocity
        trim: float       trimming factor used to offset differences in the
                            behaviour of the left and right motors, it is recommended
                            to use a value that results in the robot moving in a
                            straight line when equal forward commands are given
        k: float          motor constant, assumed equal for both motors
        limit: float      limits the final commands sent to the motors

    """

    def __init__(self, **kwargs):
        super(PWMComponent, self).__init__()
        self._pwm: PWM = PWM(**kwargs)
        # queues
        self.in_wl_wr: Queue[Tuple[OmegaLeft, OmegaRight]] = Queue()
        self.out_pwml_pwmr: Queue[Tuple[PWMLeft, PWMRight]] = Queue()

    def worker(self):
        while not self.is_shutdown:
            wl, wr = self.in_wl_wr.get()
            pwml, pwmr = self._pwm.get_wheels_duty_cycle(wl, wr)
            self.out_pwml_pwmr.put((pwml, pwmr))
