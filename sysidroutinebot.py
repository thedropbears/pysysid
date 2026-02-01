import math

import phoenix6
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from rev import SparkMax
from wpilib import DutyCycleEncoder

from constants import DioChannel, OIConstants, SparkIds, TalonIds
from subsystems.flywheel import Flywheel
from subsystems.rev_turret import RevTurret
from subsystems.swerve_drive import SwerveDrive
from subsystems.sysid_subsystem import SysidSubsystem


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

        self.turret = RevTurret(
            SparkMax(SparkIds.TURRET, SparkMax.MotorType.kBrushless),
            (1 / 5) * (25 / 145),
            True,
            math.radians(200),
            math.radians(-200),
            DutyCycleEncoder(DioChannel.TURRET_ENCODER),
            1 / ((145 / 40) * (16 / 70)),
            True,
            0.359977,
        )

        self.controller = CommandXboxController(OIConstants.CONTROLLER_PORT)

    def configureBindings(self) -> None:
        self.swerve_drive.setDefaultCommand(self.swerve_drive.defaultCommand())
        self.flywheel.setDefaultCommand(self.flywheel.defaultCommand())
        self.turret.setDefaultCommand(self.turret.defaultCommand())

        def bindSysId(subsystem: SysidSubsystem, pov: Trigger):
            (pov & self.controller.a()).whileTrue(
                subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward).onlyWhile(
                    subsystem.BeforePositiveLimit
                )
            )
            (pov & self.controller.b()).whileTrue(
                subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).onlyWhile(
                    subsystem.BeforeNegativeLimit
                )
            )
            (pov & self.controller.x()).whileTrue(
                subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward).onlyWhile(
                    subsystem.BeforePositiveLimit
                )
            )
            (pov & self.controller.y()).whileTrue(
                subsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse).onlyWhile(
                    subsystem.BeforeNegativeLimit
                )
            )

        bindSysId(self.swerve_drive, self.controller.rightBumper())
        bindSysId(self.flywheel, self.controller.leftBumper())
        bindSysId(self.turret, self.controller.rightTrigger())
