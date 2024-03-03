# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum


@enum.unique
class SparkMaxIds(enum.IntEnum):
    shooter_inclinator = 2


class OIConstants:
    kDriverControllerPort = 0
