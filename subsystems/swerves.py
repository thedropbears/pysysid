from typing import override

from phoenix6.swerve.requests import (
    SysIdSwerveRotation,
    SysIdSwerveSteerGains,
    SysIdSwerveTranslation,
)
from wpilib.sysid import SysIdRoutineLog

from generated.comp import TunerConstants, TunerSwerveDrivetrain
from subsystems.sysid_subsystem import SysidSubsystem


class Swerves(SysidSubsystem, TunerSwerveDrivetrain):
    @override
    def __init__(
        self,
    ) -> None:
        SysidSubsystem.__init__(self, step_voltage=2.5, timeout=4)

        tuner_constants = TunerConstants()
        modules = [
            tuner_constants.front_left,
            tuner_constants.front_right,
            tuner_constants.back_left,
            tuner_constants.back_right,
        ]

        TunerSwerveDrivetrain.__init__(
            self, tuner_constants.drivetrain_constants, modules
        )

        self.drive_routine = SysIdSwerveTranslation()
        self.steer_routine = SysIdSwerveSteerGains()
        self.heading_routine = SysIdSwerveRotation()

        self.chosen_routine = self.drive_routine

    @override
    def drive(self, voltage: float) -> None:
        self.set_control(
            self.chosen_routine.with_rotational_rate(voltage)
            if isinstance(self.chosen_routine, SysIdSwerveRotation)
            else self.chosen_routine.with_volts(voltage)
        )

    @override
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        # just have hoot stuff instead
        pass
