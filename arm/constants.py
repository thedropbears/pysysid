import enum


@enum.unique
class SparkId(enum.IntEnum):
    intake_deploy_leader = 4
    intake_deploy_follower = 5
    wrist_motor = 5


class OIConstants:
    kDriverControllerPort = 0
