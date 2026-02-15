import enum


@enum.unique
class TalonIds(enum.IntEnum):
    DRIVE_FL = 1
    STEER_FL = 5

    DRIVE_RL = 2
    STEER_RL = 6

    DRIVE_RR = 3
    STEER_RR = 7

    DRIVE_FR = 4
    STEER_FR = 8

    FLYWHEEL = 9

    TURRET = 10


@enum.unique
class CancoderIds(enum.IntEnum):
    SWERVE_FL = 1
    SWERVE_RL = 2
    SWERVE_RR = 3
    SWERVE_FR = 4


@enum.unique
class SparkIds(enum.IntEnum):
    TURRET = 2


@enum.unique
class DioChannel(enum.IntEnum):
    TURRET_ENCODER = 1


class OIConstants:
    CONTROLLER_PORT = 0
