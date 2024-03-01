# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum


@enum.unique
class TalonIds(enum.IntEnum):
    shooter_flywheel_left = 9
    shooter_flywheel_right = 10


class OIConstants:
    kDriverControllerPort = 0
