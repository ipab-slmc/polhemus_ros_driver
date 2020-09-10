#!/usr/bin/env python
#
# Copyright (C) 2019 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from math import pi

class SourcePoseCalibration(object):
    def __init__(self):
        self.CONST_SOURCE_ORIENTATION = [0, 0.1, -pi]

    def calibrate(self, knuckle_thickness, knuckle_to_source):
        return [-(knuckle_to_source + 0.008), 0, knuckle_thickness / 2 + 0.016] + self.CONST_SOURCE_ORIENTATION

    # Currently orientation does not affect decalibration
    def decalibrate(self, x, y, z):
        knuckle_thickness = (z - 0.016) * 2
        knuckle_to_source = -(x + 0.008)
        return [knuckle_thickness, knuckle_to_source]