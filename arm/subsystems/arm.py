import math

import rev
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import sysid


class Arm(Subsystem):
    def __init__(
        self,
        motor: rev.SparkMax,
        follower: rev.SparkMax,
        *,
        oppose_leader: bool,
        gearing: float,
        name: str | None = None,
    ) -> None:
        self.motor = motor
        config = self._create_smax_config(gearing)
        self._configure_ephemeral(motor, config)
        self.encoder = motor.getEncoder()
        if name is not None:
            self.setName(name)

        if True:
            self.follower = follower
            self.follower_encoder = follower.getEncoder()
            follower_config = self._create_smax_config(gearing)
            follower_config.follow(motor, invert=oppose_leader)
            self._configure_ephemeral(follower, follower_config)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(self.motor.setVoltage, self.log, self),
        )

    @classmethod
    def _create_smax_config(cls, gearing: float) -> rev.SparkMaxConfig:
        config = rev.SparkMaxConfig()
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        # Measure position in rad and velocity in rad/s.
        output_rad_per_motor_rev = gearing * math.tau
        config.encoder.positionConversionFactor(output_rad_per_motor_rev)
        config.encoder.velocityConversionFactor(output_rad_per_motor_rev / 60)

        return config

    @classmethod
    def _configure_ephemeral(cls, motor: rev.SparkBase, config: rev.SparkBaseConfig):
        motor.configure(
            config,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kNoPersistParameters,
        )

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
