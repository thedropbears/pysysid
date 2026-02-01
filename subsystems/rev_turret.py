import math
import typing

from rev import SparkMax, SparkMaxConfig
from wpilib import DutyCycleEncoder, sysid
from wpimath import units

from subsystems.sysid_subsystem import SysidSubsystem
from utilities.rev import configure_spark_ephemeral


class RevTurret(SysidSubsystem):
    def __init__(
        self,
        motor: SparkMax,
        motor_to_mechanism_gearing: float,
        invert_motor: bool,
        positive_limit: units.radians,
        negative_limit: units.radians,
        absolute_encoder: DutyCycleEncoder,
        encoder_to_mechanism_gearing: float,
        invert_absolute_encoder: bool,
        encoder_offset: float,
    ):
        super().__init__(ramp_rate=0.5, step_voltage=3.5, timeout=5.0)
        self.motor = motor

        config = SparkMaxConfig()
        config.encoder.positionConversionFactor(motor_to_mechanism_gearing * math.tau)
        config.encoder.velocityConversionFactor(
            1 / 60 * motor_to_mechanism_gearing * math.tau
        )
        config.inverted(invert_motor)
        self.positive_limit = positive_limit
        config.softLimit.forwardSoftLimit(positive_limit)
        config.softLimit.forwardSoftLimitEnabled(True)
        self.negative_limit = negative_limit
        config.softLimit.reverseSoftLimit(negative_limit)
        config.softLimit.reverseSoftLimitEnabled(True)

        configure_spark_ephemeral(self.motor, config)

        self.relative_encoder = self.motor.getEncoder()
        self.absolute_encoder = absolute_encoder
        self.absolute_encoder.setInverted(invert_absolute_encoder)
        self.encoder_offset = encoder_offset
        self.encoder_to_mechanism_gearing = encoder_to_mechanism_gearing

        self.relative_encoder.setPosition(
            (absolute_encoder.get() - encoder_offset)
            * encoder_to_mechanism_gearing
            * math.tau
        )

    def sync_encoder(self) -> None:
        self.relative_encoder.setPosition(self.read_absolute_position())

    def read_absolute_position(self) -> units.radians:
        return (
            (self.absolute_encoder.get() - self.encoder_offset)
            * self.encoder_to_mechanism_gearing
            * math.tau
        )

    # Tell SysId how to plumb the driving voltage to the motors.
    @typing.override
    def drive(self, voltage: units.volts) -> None:
        self.motor.setVoltage(voltage)

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    @typing.override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.motor.getAppliedOutput())
            .position(self.relative_encoder.getPosition())
            .velocity(self.relative_encoder.getVelocity())
        )

    @typing.override
    def BeforePositiveLimit(self) -> bool:
        return self.relative_encoder.getPosition() < self.positive_limit

    @typing.override
    def BeforeNegativeLimit(self) -> bool:
        return self.relative_encoder.getPosition() > self.negative_limit
