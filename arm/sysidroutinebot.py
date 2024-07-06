import rev
from commands2 import Command
from commands2.button import CommandXboxController
from commands2.sysid import SysIdRoutine

from subsystems.arm import Arm

from constants import OIConstants, SparkId


class SysIdRoutineBot:
    """This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.arm = Arm(
            self._create_neo(SparkId.intake_deploy_leader),
            self._create_neo(SparkId.intake_deploy_follower),
            oppose_leader=True,
            gearing=(1 / 5) * (1 / 3) * (24 / 72),
        )

        # The driver's controller
        self.controller = CommandXboxController(OIConstants.kDriverControllerPort)

    @staticmethod
    def _create_neo(id: int) -> rev.CANSparkMax:
        return rev.CANSparkMax(id, rev.CANSparkMax.MotorType.kBrushless)

    def configureBindings(self) -> None:
        """Use this method to define bindings between conditions and commands. These are useful for
        automating robot behaviors based on button and sensor input.

        Should be called during :meth:`.Robot.robotInit`.

        Event binding methods are available on the :class:`.Trigger` class.
        """

        # Control the drive with split-stick arcade controls
        self.arm.setDefaultCommand(self.arm.defaultCommand())

        # Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        # once.

        self.controller.a().whileTrue(
            self.arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        self.controller.b().whileTrue(
            self.arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        self.controller.x().whileTrue(
            self.arm.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        self.controller.y().whileTrue(
            self.arm.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

    def getAutonomousCommand(self) -> Command:
        """Use this to define the command that runs during autonomous.

        Scheduled during :meth:`.Robot.autonomousInit`.
        """

        # Do nothing
        return self.arm.run(lambda: None)
