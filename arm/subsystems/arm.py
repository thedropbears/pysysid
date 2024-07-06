import math

import rev

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import sysid


class Arm(Subsystem):
    def __init__(
        self,
        motor: rev.CANSparkBase,
        follower: rev.CANSparkBase,
        *,
        oppose_leader: bool,
        gearing: float,
        name: str | None = None,
    ) -> None:
        self.motor = motor
        self.encoder = self._init_encoder(self.motor, gearing)
        if name is not None:
            self.setName(name)

        self.motor.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        if True:
            self.follower = follower
            self.follower_encoder = self._init_encoder(follower, gearing)
            follower.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
            # follower.setInverted(oppose_leader)
            follower.follow(self.motor, invert=oppose_leader)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(self.motor.setVoltage, self.log, self),
        )

    def _init_encoder(
        self, motor: rev.CANSparkBase, gearing: float
    ) -> rev.SparkRelativeEncoder:
        encoder = motor.getEncoder()

        # Measure position in rad and velocity in rad/s.
        output_rad_per_motor_rev = gearing * math.tau
        encoder.setPositionConversionFactor(output_rad_per_motor_rev)
        encoder.setVelocityConversionFactor(output_rad_per_motor_rev / 60)

        return encoder

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.motor.get() * self.motor.getBusVoltage())
            .position(self.encoder.getPosition())
            .velocity(self.encoder.getVelocity())
        )
        if True:
            (
                sys_id_routine.motor("follower")
                .voltage(self.follower.get() * self.follower.getBusVoltage())
                .position(self.follower_encoder.getPosition())
                .velocity(self.follower_encoder.getVelocity())
            )

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
