#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

"""
This is a demo program showing the use of OpenCV to do vision processing. The image is acquired
from an HTTP camera, then a rectangle is put on the image and sent to the dashboard. OpenCV has
many methods for different types of processing.
"""

import ntcore
import numpy
import cscore
from cscore import CameraServer
import cv2


#
# This code will work both on a RoboRIO and on other platforms. The exact mechanism
# to run it differs depending on whether you’re on a RoboRIO or a coprocessor
#
# https://robotpy.readthedocs.io/en/stable/vision/code.html


def main():
    # Start capturing images
    cameras: list[cscore.UsbCamera] = []
    for i in range(1):
        cameras.append(CameraServer.startAutomaticCapture(name=f"USB Camera {i}", path=f"/dev/video{i}"))
    # Set the resolution
    #_ = camera.setResolution(640, 480)
    for camera in cameras:
        camera.setResolution(640, 480)
        camera.setFPS(30)
        camera.setPixelFormat(cscore.VideoMode.PixelFormat.kYUYV)

    while True:
        pass