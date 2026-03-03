import math

import phoenix6
from commands2 import DeferredCommand
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from constants import OIConstants, TalonIds
from subsystems.flywheel import Flywheel
from subsystems.swerve_drive import SwerveDrive
from subsystems.sysid_subsystem import SysidSubsystem
from subsystems.talon_turret import TalonTurret


class SysIdRoutineBot:
    def __init__(self) -> None:

        # This is only for the drive motors
        # Make sure to have configured the proper gear reduction and gains internally
        self.swerve_drive = SwerveDrive()

        # This can be applied to general flywheel systems
        #  - shooter
        #  - swerve steer
        # if using a swerve steer make sure to comment out references to the drive
        # system to avoid double motor initialisation
        self.flywheel = Flywheel(
            phoenix6.hardware.TalonFX(TalonIds.FLYWHEEL),
            gearing=1 / ((14 / 50) * (10 / 60)),
        )

        """self.turret = RevTurret(
            SparkMax(SparkIds.TURRET, SparkMax.MotorType.kBrushless),
            (1 / 5) * (25 / 145),
            True,
            math.radians(200),
            math.radians(-200),
            DutyCycleEncoder(DioChannel.TURRET_ENCODER),
            1 / ((145 / 40) * (16 / 70)),
            True,
            0.359977,
        )"""

        self.turret = TalonTurret(
            phoenix6.hardware.TalonFXS(TalonIds.TURRET),
            1 / ((1 / 5) * (40 / 200) * math.tau),
            math.radians(200),
            math.radians(-200),
        )

        self.controller = CommandXboxController(OIConstants.CONTROLLER_PORT)

    def configureBindings(self) -> None:
        self.swerve_drive.setDefaultCommand(self.swerve_drive.defaultCommand())
        self.flywheel.setDefaultCommand(self.flywheel.defaultCommand())
        self.turret.setDefaultCommand(self.turret.defaultCommand())

        def bindSysId(subsystem: SysidSubsystem, pov: Trigger):
            (
                pov
                & self.controller.a()
                & Trigger(lambda: not subsystem.atPositiveLimit())
            ).whileTrue(
                DeferredCommand(
                    lambda: subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                    subsystem,
                )
            )

            (
                pov
                & self.controller.b()
                & Trigger(lambda: not subsystem.atNegativeLimit())
            ).whileTrue(
                DeferredCommand(
                    lambda: subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                    subsystem,
                )
            )

            (
                pov
                & self.controller.x()
                & Trigger(lambda: not subsystem.atPositiveLimit())
            ).whileTrue(
                DeferredCommand(
                    lambda: subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward),
                    subsystem,
                )
            )

            (
                pov
                & self.controller.y()
                & Trigger(lambda: not subsystem.atNegativeLimit())
            ).whileTrue(
                DeferredCommand(
                    lambda: subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                    subsystem,
                )
            )

        bindSysId(self.swerve_drive, self.controller.rightBumper())
        bindSysId(self.flywheel, self.controller.leftBumper())
        bindSysId(self.turret, self.controller.rightTrigger())
