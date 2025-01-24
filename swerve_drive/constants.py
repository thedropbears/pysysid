# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum


@enum.unique
class TalonIds(enum.IntEnum):
    drive_1 = 1
    steer_1 = 5

    drive_2 = 2
    steer_2 = 6

    drive_3 = 3
    steer_3 = 7

    drive_4 = 4
    steer_4 = 8


@enum.unique
class CancoderIds(enum.IntEnum):
    swerve_1 = 1
    swerve_2 = 2
    swerve_3 = 3
    swerve_4 = 4


class OIConstants:
    kDriverControllerPort = 0
