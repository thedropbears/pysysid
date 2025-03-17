import math

import rev
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import sysid


class Arm(Subsystem):
    def __init__(
        self,
        motor: rev.SparkMax,
        follower: rev.SparkMax | None,
        *,
        oppose_leader: bool,
        gearing: float,
        lower_limit: float,
        upper_limit: float,
        motor_inverted: bool,
        name: str | None = None,
    ) -> None:
        self.motor = motor
        config = self._create_smax_config(gearing, upper_limit, lower_limit, motor_inverted)
        self._configure_ephemeral(motor, config)
        self.encoder = motor.getEncoder()
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit
        if name is not None:
            self.setName(name)

        self.follower = follower

        if self.follower is not None:
            
            self.follower_encoder = self.follower.getEncoder()
            follower_config = self._create_smax_config(gearing, upper_limit, lower_limit, motor_inverted)
            follower_config.follow(motor, invert=oppose_leader)
            self._configure_ephemeral(self.follower, follower_config)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(rampRate=3, stepVoltage=7),
            SysIdRoutine.Mechanism(self.motor.setVoltage, self.log, self),
        )

    @classmethod
    def _create_smax_config(cls, gearing: float, upper_limit: float, lower_limit: float, inverted: bool) -> rev.SparkMaxConfig:
        config = rev.SparkMaxConfig()
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        config.softLimit.reverseSoftLimitEnabled(True)
        config.softLimit.reverseSoftLimit(lower_limit)
        config.softLimit.forwardSoftLimitEnabled(True)
        config.softLimit.forwardSoftLimit(upper_limit)
        config.inverted(inverted)
        
        # Measure position in rad and velocity in rad/s.
        output_rad_per_motor_rev = (1/gearing) * math.tau
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
        if self.follower is not None:
            (
                sys_id_routine.motor("follower")
                .voltage(self.follower.get() * self.follower.getBusVoltage())
                .position(self.follower_encoder.getPosition())
                .velocity(self.follower_encoder.getVelocity())
            )


    def below_upper_limit(self) -> bool:
        return self.encoder.getPosition() < self.upper_limit
    
    def above_lower_limit(self) -> bool:
        return self.encoder.getPosition() > self.lower_limit

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
