# %% [markdown]
# # Final Project

# %% [markdown]
# 
# ## TODO: Add project description here

# %%
%load_ext autoreload
%autoreload 2

import os

import numpy as np
import cv2
import matplotlib.pyplot as plt

VEHICLE_NAME: str = "kizzy"

# %%
## Real Duckiebot - KIZZY

import os
from duckietown.types import CameraParameters

if VEHICLE_NAME == "kizzy":
    CAMERA_PARAMETERS: CameraParameters = {
        "width": 640,
        "height": 480,
        "K": np.reshape(
            [
                275.73276038498926,
                0.0,
                319.6766909474663,
                0.0,
                280.1548497747509,
                273.2555774703665,
                0.0,
                0.0,
                1.0,
            ],
            (3, 3),
        ),
        "D": [
            -0.186769174617111,
            0.021237828783060932,
            -0.010063453683894804,
            0.0020176247114933833,
            0.0,
        ],
        "P": np.reshape(
            [
                181.33279418945312,
                0.0,
                321.308646777532,
                0.0,
                0.0,
                230.59254455566406,
                277.2216409396642,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ],
            (3, 4),
        ),
        "H": np.reshape(
            [
                # 8.56148231e-03,
                # 2.22480148e-01,
                # 4.24318934e-01,
                # -5.67022044e-01,
                # -1.13258040e-03,
                # 6.81113839e-04,
                # 5.80917161e-02,
                # 4.35079347e00,
                # 1.0,

                0.0219712,
                0.09045404,
                0.38400369,
                -0.44468068,
                0.01655425,
                0.01499358,
                0.20360817,
                2.9932356,
                1.        
            ],
            (3, 3),
        ),
    }

# %%
# Sensor - Camera

from duckietown.components.duckiebot import CameraDriverComponent

camera: CameraDriverComponent = CameraDriverComponent(vehicle_name=VEHICLE_NAME)

# %%
# Image Cropping

from duckietown.components.lane_following import ImageCropComponent

image_crop: ImageCropComponent = ImageCropComponent(parameters=CAMERA_PARAMETERS)

# %%
# Line Detector

from duckietown.components.lane_following import LineDetectorComponent

line_detector: LineDetectorComponent = LineDetectorComponent()

# %%
# Lane Filter

from duckietown.components.lane_following import LaneFilterComponent

lane_filter: LaneFilterComponent = LaneFilterComponent(camera_parameters=CAMERA_PARAMETERS)

# %%
# Lane Controller

from duckietown.components.lane_following import LaneControllerComponent

lane_controller: LaneControllerComponent = LaneControllerComponent()

# %%
# Inverse Kinematics

from duckietown.components.lane_following import InverseKinematicsComponent

inverse_kinematics: InverseKinematicsComponent = InverseKinematicsComponent()

# %%
# PWM Mapper

from duckietown.components.lane_following import PWMComponent

pwm: PWMComponent = PWMComponent()

# %%
# Wheels Driver

from duckietown.components.duckiebot import MotorsDriverComponent

motors: MotorsDriverComponent = MotorsDriverComponent(vehicle_name=VEHICLE_NAME)

# %%
# Connect components

image_crop.in_bgr.wants(camera.out_bgr)

line_detector.in_bgr.wants(image_crop.out_bgr)

lane_filter.in_lines.wants(line_detector.out_lines)
lane_filter.in_command_time.wants(motors.out_command_time)
lane_filter.in_v_omega.wants(lane_controller.out_v_omega)

lane_controller.in_d_phi.wants(lane_filter.out_d_phi)

inverse_kinematics.in_v_omega.wants(lane_controller.out_v_omega)

pwm.in_wl_wr.wants(inverse_kinematics.out_wl_wr)

motors.in_pwml_pwmr.wants(pwm.out_pwml_pwmr)

# %%
# Rendering components

from duckietown.components.rendering import ImageRendererComponent, TextRendererComponent

# define components
detections: ImageRendererComponent = ImageRendererComponent()
segments: ImageRendererComponent = ImageRendererComponent()
belief: ImageRendererComponent = ImageRendererComponent()
pwm_signals: TextRendererComponent = TextRendererComponent()

# %%
import time
from typing import List

from duckietown.components.base import Component
from duckietown.system import System


# connect rendering components
# detections.in_image.wants(camera.out_bgr)
# detections.in_image.wants(line_detector.out_lines_yellow_image)
# segments.in_image.wants(lane_filter.out_segments_image)
# belief.in_image.wants(lane_filter.out_belief_image)
pwm_signals.in_data.wants(pwm.out_pwml_pwmr)

# list of components to run
components: List[Component] = [
    camera,
    image_crop,
    line_detector,
    lane_filter,
    lane_controller,
    inverse_kinematics,
    pwm,
    motors,
    # -- rendering
    detections,
    segments,
    belief,
    pwm_signals
]

# create system
system: System = System(components)

# run system (NOTE: this is blocking)
system.run()

# %%



