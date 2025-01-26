import enum


@enum.unique
class SparkId(enum.IntEnum):
    wrist_motor = 5


class OIConstants:
    kDriverControllerPort = 0
