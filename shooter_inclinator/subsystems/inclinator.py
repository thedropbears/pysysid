# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import rev

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import sysid

from wpimath.units import volts

FollowerDescriptor = tuple[rev.CANSparkMax, bool]


class Inclinator(Subsystem):
    MAX_INCLINE_ANGLE = 1.045  # ~60 degrees
    MIN_INCLINE_ANGLE = 0.354  # ~20 degrees
    INCLINATOR_TOLERANCE = math.radians(1)
    INCLINATOR_OFFSET = 3.972 - math.radians(60)

    INCLINATOR_SCALE_FACTOR = math.tau  # rps -> radians

    INCLINATOR_GEAR_RATIO = 18 / 24 * 26 / 300
    INCLINATOR_POSITION_CONVERSION_FACTOR = (
        INCLINATOR_GEAR_RATIO * math.tau
    )  # motor rotations -> mech rads
    INCLINATOR_VELOCITY_CONVERSION_FACTOR = (
        INCLINATOR_POSITION_CONVERSION_FACTOR / 60
    )  # rpm -> radians/s

    def __init__(
        self,
        inclinator_motor: rev.CANSparkMax,
        name: str | None = None,
    ) -> None:
        self.inclinator = inclinator_motor
        self.inclinator.setInverted(True)
        self.inclinator.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

        self.inclinator_encoder = self.inclinator.getEncoder()
        self.inclinator_encoder.setVelocityConversionFactor(
            self.INCLINATOR_VELOCITY_CONVERSION_FACTOR
        )
        self.inclinator_encoder.setPositionConversionFactor(
            self.INCLINATOR_POSITION_CONVERSION_FACTOR
        )

        self.inclinator_setpoint = self.MIN_INCLINE_ANGLE
        self.inclinator_encoder.setPosition(self.inclinator_setpoint)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(recordState=self.recordState),
            SysIdRoutine.Mechanism(self.drive, self.log, self),
        )

        self.logger_inited = False

    # Tell SysId how to plumb the driving voltage to the motors.
    def drive(self, voltage: volts) -> None:
        self.inclinator.setVoltage(voltage)

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.inclinator.getBusVoltage())
            .position(self.inclinator_encoder.getPosition())
            .velocity(self.inclinator_encoder.getVelocity())
        )

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
