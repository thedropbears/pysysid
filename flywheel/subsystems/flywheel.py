# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import phoenix6
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from phoenix6.configs import FeedbackConfigs, MotorOutputConfigs
from phoenix6.controls import Follower, VoltageOut
from phoenix6.signals import NeutralModeValue
from wpilib import sysid
from wpimath.units import volts

FollowerDescriptor = tuple[phoenix6.hardware.TalonFX, bool]


class Flywheel(Subsystem):
    def __init__(
        self,
        flywheel_motor: phoenix6.hardware.TalonFX,
        *followers: FollowerDescriptor,
        gearing: float,
        name: str | None = None,
    ) -> None:
        self.flywheel = flywheel_motor
        self.followers = [motor for motor, _ in followers]
        if name is not None:
            self.setName(name)

        flywheel_motor_config = MotorOutputConfigs()
        flywheel_motor_config.neutral_mode = NeutralModeValue.COAST
        feedback_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(gearing)

        flywheel_config = self.flywheel.configurator
        flywheel_config.apply(flywheel_motor_config)
        flywheel_config.apply(feedback_config)

        for motor, oppose_leader in followers:
            motor.configurator.apply(flywheel_motor_config)
            motor.configurator.apply(feedback_config)
            motor.set_control(Follower(self.flywheel.device_id, oppose_leader))

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(recordState=self.recordState),
            SysIdRoutine.Mechanism(self.drive, self.log, self),
        )

        self.logger_inited = False

    # Tell SysId how to plumb the driving voltage to the motors.
    def drive(self, voltage: volts) -> None:
        self.flywheel.set_control(VoltageOut(voltage))

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.flywheel.get_motor_voltage().value)
            .position(self.flywheel.get_position().value)
            .velocity(self.flywheel.get_velocity().value)
        )
        for motor in self.followers:
            (
                sys_id_routine.motor(f"follower-{motor.device_id}")
                .voltage(motor.get_motor_voltage().value)
                .position(motor.get_position().value)
                .velocity(motor.get_velocity().value)
            )

    def recordState(self, state: sysid.State) -> None:
        if not self.logger_inited:
            SignalLogger.start()
            self.logger_inited = True

        SignalLogger.write_string(
            f"sysid-test-state-{self.getName()}",
            sysid.SysIdRoutineLog.stateEnumToString(state),
        )
        self.sys_id_routine.recordState(state)

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
