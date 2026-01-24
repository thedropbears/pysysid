import phoenix6
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from constants import OIConstants, TalonIds
from subsystems.flywheel import Flywheel
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

        self.controller = CommandXboxController(OIConstants.CONTROLLER_PORT)

    def configureBindings(self) -> None:
        # self.swerve_drive.setDefaultCommand(self.swerve_drive.defaultCommand())
        self.flywheel.setDefaultCommand(self.flywheel.defaultCommand())

        def bindSysId(subsystem: SysidSubsystem, pov: Trigger):
            (pov & self.controller.a()).whileTrue(
                subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            )
            (pov & self.controller.b()).whileTrue(
                subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            )
            (pov & self.controller.x()).whileTrue(
                subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward)
            )
            (pov & self.controller.y()).whileTrue(
                subsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
            )

        bindSysId(self.swerve_drive, self.controller.rightBumper())
        bindSysId(self.flywheel, self.controller.leftBumper())
