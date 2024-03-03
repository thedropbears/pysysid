# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from rev import CANSparkMax

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import sysid

from wpimath.units import volts


FollowerDescriptor = tuple[CANSparkMax, bool]


class IntakeArm(Subsystem):
    GEAR_RATIO = (1 / 5) * (1 / 4) * (24 / 72)
    MOTOR_REV_TO_SHAFT_RADIANS = GEAR_RATIO * math.tau
    MOTOR_RPM_TO_SHAFT_RAD_PER_SEC = MOTOR_REV_TO_SHAFT_RADIANS / 60

    SHAFT_REV_RETRACT_HARD_LIMIT = 0.0
    SHAFT_REV_DEPLOY_HARD_LIMIT = 1.353

    def __init__(
        self,
        deploy_motor_leader: CANSparkMax,
        *followers: FollowerDescriptor,
        name: str | None = None,
    ) -> None:
        self.intake_arm = deploy_motor_leader
        self.intake_arm.setInverted(True)
        self.intake_arm.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self.followers = [(motor, motor.getEncoder()) for motor, _ in followers]
        if name is not None:
            self.setName(name)

        self.deploy_encoder = self.deploy_motor_l.getEncoder()
        self.deploy_encoder.setVelocityConversionFactor(
            self.MOTOR_RPM_TO_SHAFT_RAD_PER_SEC
        )
        self.deploy_encoder.setPositionConversionFactor(self.MOTOR_REV_TO_SHAFT_RADIANS)

        for pair, oppose_leader in followers:
            motor, encoder = pair
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
            motor.follow(self.intake_arm, oppose_leader)

        self.deploy_setpoint = self.SHAFT_REV_RETRACT_HARD_LIMIT
        self.deploy_encoder.setPosition(self.deploy_setpoint)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(recordState=self.recordState),
            SysIdRoutine.Mechanism(self.drive, self.log, self),
        )

        self.logger_inited = False

    # Tell SysId how to plumb the driving voltage to the motors.
    def drive(self, voltage: volts) -> None:
        self.intake_arm.setVoltage(voltage)

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.intake_arm.getBusVoltage())
            .position(self.deploy_encoder.getPosition())
            .velocity(self.deploy_encoder.getVelocity())
        )
        for pair in self.followers:
            motor, encoder = pair
            (
                sys_id_routine.motor(f"follower-{motor.device_id}")
                .voltage(motor.getBusVoltage())
                .position(encoder.getPosition())
                .velocity(encoder.getVelocity())
            )

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
