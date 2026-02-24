import math

import phoenix6
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from wpilib import DutyCycleEncoder

from constants import DioChannel, OIConstants, TalonIds
from subsystems.flywheel import Flywheel
from subsystems.swerve_drive import SwerveDrive
from subsystems.sysid_subsystem import SysidSubsystem
from subsystems.talon_arm import TalonArm
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

        self.talon_arm = TalonArm(
            phoenix6.hardware.TalonFX(TalonIds.INTAKE_DEPLOYER_LEFT),
            (phoenix6.hardware.TalonFX(TalonIds.INTAKE_DEPLOYER_RIGHT), True),
            invert_motor=False,
            motor_to_mechanism_gearing=(5 / 1) * (26 / 50),
            absolute_encoder=DutyCycleEncoder(DioChannel.INTAKE_DEPLOYER_ENCODER),
            encoder_offset=0,
            positive_limit=math.radians(90),
            negative_limit=math.radians(0),
        )

        self.controller = CommandXboxController(OIConstants.CONTROLLER_PORT)

    def configureBindings(self) -> None:
        self.swerve_drive.setDefaultCommand(self.swerve_drive.defaultCommand())
        self.flywheel.setDefaultCommand(self.flywheel.defaultCommand())
        self.turret.setDefaultCommand(self.turret.defaultCommand())

        def bindSysId(subsystem: SysidSubsystem, pov: Trigger):
            (pov & self.controller.a()).whileTrue(
                subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward).onlyWhile(
                    subsystem.beforePositiveLimit
                )
            )
            (pov & self.controller.b()).whileTrue(
                subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).onlyWhile(
                    subsystem.beforeNegativeLimit
                )
            )
            (pov & self.controller.x()).whileTrue(
                subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward).onlyWhile(
                    subsystem.beforePositiveLimit
                )
            )
            (pov & self.controller.y()).whileTrue(
                subsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse).onlyWhile(
                    subsystem.beforeNegativeLimit
                )
            )

        bindSysId(self.swerve_drive, self.controller.rightBumper())
        bindSysId(self.flywheel, self.controller.leftBumper())
        bindSysId(self.turret, self.controller.rightTrigger())
        bindSysId(self.talon_arm, self.controller.leftTrigger())
