# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib import sysid

from phoenix6 import SignalLogger
from phoenix6.hardware import TalonFX
from phoenix6.configs import FeedbackConfigs, MotorOutputConfigs
from phoenix6.controls import VoltageOut
from phoenix6.signals import NeutralModeValue

from wpimath.units import volts

from constants import TalonIds


class Flywheel(Subsystem):
    FLYWHEEL_GEAR_RATIO = 24.0 / 18.0

    def __init__(self) -> None:
        self.flywheel = TalonFX(TalonIds.shooter_flywheel)

        flywheel_config = self.flywheel.configurator
        flywheel_motor_config = MotorOutputConfigs()
        flywheel_motor_config.neutral_mode = NeutralModeValue.COAST
        flywheel_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.FLYWHEEL_GEAR_RATIO
        )
        flywheel_config.apply(flywheel_motor_config)
        flywheel_config.apply(flywheel_gear_ratio_config)

        # Tell SysId how to plumb the driving voltage to the motors.
        def drive(voltage: volts) -> None:
            voltage_request = VoltageOut(voltage)
            self.flywheel.set_control(voltage_request)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(recordState=self.recordState),
            SysIdRoutine.Mechanism(drive, self.log, self),
        )

        self.logger_inited = False

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        sys_id_routine.motor("flywheel").voltage(
            self.flywheel.get_motor_voltage().value
        ).position(self.flywheel.get_position().value).velocity(
            self.flywheel.get_velocity().value
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
