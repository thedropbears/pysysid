from typing import override

from phoenix6 import swerve
from phoenix6.swerve.requests import (
    SysIdSwerveRotation,
    SysIdSwerveSteerGains,
    SysIdSwerveTranslation,
)

from subsystems.sysid_subsystem import SysidSubsystem
from swerves.tuner_constants import TunerSwerveDrivetrain


class Swerves(SysidSubsystem, TunerSwerveDrivetrain):
    @override
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        SysidSubsystem.__init__(self, step_voltage=4)
        TunerSwerveDrivetrain.__init__(self, drivetrain_constants, modules)

        self.drive_routine = SysIdSwerveTranslation()
        self.steer_routine = SysIdSwerveSteerGains()
        self.heading_routine = SysIdSwerveRotation()
        # TODO actually figure out what swerve rotation does

        self.chosen_routine = self.drive_routine

    @override
    def drive(self, voltage: float) -> None:
        self.set_control(
            self.chosen_routine.with_rotational_rate(voltage)
            if isinstance(self.chosen_routine, SysIdSwerveRotation)
            else self.chosen_routine.with_volts(voltage)
        )
