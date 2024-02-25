# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import phoenix6
from commands2 import Command
from commands2.button import CommandXboxController
from commands2.sysid import SysIdRoutine

from subsystems.flywheel import Flywheel

from constants import OIConstants, TalonIds


class SysIdRoutineBot:
    """This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.flywheel = Flywheel(
            phoenix6.hardware.TalonFX(TalonIds.shooter_flywheel), gearing=24.0 / 18.0
        )

        # The driver's controller
        self.controller = CommandXboxController(OIConstants.kDriverControllerPort)

    def configureBindings(self) -> None:
        """Use this method to define bindings between conditions and commands. These are useful for
        automating robot behaviors based on button and sensor input.

        Should be called during :meth:`.Robot.robotInit`.

        Event binding methods are available on the :class:`.Trigger` class.
        """

        # Control the drive with split-stick arcade controls
        self.flywheel.setDefaultCommand(self.flywheel.defaultCommand())

        # Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        # once.

        self.controller.a().whileTrue(
            self.flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        self.controller.b().whileTrue(
            self.flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        self.controller.x().whileTrue(
            self.flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        self.controller.y().whileTrue(
            self.flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

    def getAutonomousCommand(self) -> Command:
        """Use this to define the command that runs during autonomous.

        Scheduled during :meth:`.Robot.autonomousInit`.
        """

        # Do nothing
        return self.flywheel.run(lambda: None)
