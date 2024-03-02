# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum


@enum.unique
class SparkMaxIds(enum.IntEnum):
    intake_deploy_l = 4
    intake_deploy_r = 5


class OIConstants:
    kDriverControllerPort = 0
