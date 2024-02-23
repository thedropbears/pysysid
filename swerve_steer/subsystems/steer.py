# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog


from phoenix6.hardware import TalonFX
from phoenix6.configs import FeedbackConfigs, MotorOutputConfigs
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import VoltageOut

from wpimath.units import volts

from constants import TalonIds


class Steer(Subsystem):
    STEER_GEAR_RATIO = (14 / 50) * (10 / 60)

    def __init__(self) -> None:
        self.steer = TalonFX(TalonIds.steer_1)

        steer_motor_config = MotorOutputConfigs()
        steer_motor_config.neutral_mode = NeutralModeValue.BRAKE

        steer_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / self.STEER_GEAR_RATIO
        )

        steer_config = self.steer.configurator
        steer_config.apply(steer_motor_config)
        steer_config.apply(steer_gear_ratio_config)

        # Tell SysId how to plumb the driving voltage to the motors.
        # We should only be dispatching to one in the case of steer to reduce potential scrubbing as the platform fights itself
        def drive(voltage: volts) -> None:
            voltage_request = VoltageOut(voltage)
            self.steer.set_control(voltage_request)

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(drive, self.log, self),
        )

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:

        sys_id_routine.motor("steer").voltage(
            self.steer.get_motor_voltage().value
        ).position(self.steer.get_position().value).velocity(
            self.steer.get_velocity().value
        )

    def defaultCommand(self) -> Command:
        return self.run(lambda: None)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
