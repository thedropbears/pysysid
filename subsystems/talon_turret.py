import math
import typing

from phoenix6.configs import (
    CANcoderConfiguration,
    CommutationConfigs,
    ExternalFeedbackConfigs,
    MagnetSensorConfigs,
    MotorOutputConfigs,
    TalonFXSConfiguration,
)
from phoenix6.controls import VoltageOut
from phoenix6.hardware import CANcoder, TalonFXS
from phoenix6.signals import (
    ExternalFeedbackSensorSourceValue,
    InvertedValue,
    MotorArrangementValue,
    NeutralModeValue,
    SensorDirectionValue,
)
from wpilib import sysid
from wpimath import units

from subsystems.sysid_subsystem import SysidSubsystem

rotations = float


class TalonTurret(SysidSubsystem):
    def __init__(
        self,
        motor: TalonFXS,
        rotor_to_sensor_gearing: float,
        absolute_encoder: CANcoder,
        sensor_to_mechanism_gearing: float,
        encoder_offset: rotations,
        positive_limit: units.radians,
        negative_limit: units.radians,
    ) -> None:
        super().__init__(ramp_rate=0.5, step_voltage=1.75, timeout=10.0)

        self.motor = motor
        self.absolute_encoder = absolute_encoder
        self.positive_limit = positive_limit / math.tau
        self.negative_limit = negative_limit / math.tau

        motor_output_config = (
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.BRAKE)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        )
        external_feedback_config = (
            ExternalFeedbackConfigs()
            .with_rotor_to_sensor_ratio(rotor_to_sensor_gearing)
            .with_sensor_to_mechanism_ratio(sensor_to_mechanism_gearing)
            .with_external_feedback_sensor_source(
                ExternalFeedbackSensorSourceValue.REMOTE_CANCODER
            )
            .with_feedback_remote_sensor_id(absolute_encoder.device_id)
        )

        # limit_switch_config = (
        #     SoftwareLimitSwitchConfigs()
        #     .with_forward_soft_limit_threshold(positive_limit)
        #     .with_forward_soft_limit_enable(True)
        #     .with_reverse_soft_limit_threshold(negative_limit)
        #     .with_reverse_soft_limit_enable(True)
        # )

        self.motor.configurator.apply(
            TalonFXSConfiguration()
            .with_motor_output(motor_output_config)
            .with_external_feedback(external_feedback_config)
            .with_commutation(
                CommutationConfigs().with_motor_arrangement(
                    MotorArrangementValue.MINION_JST
                )
            )
            # .with_software_limit_switch(limit_switch_config)
        )

        self.absolute_encoder.configurator.apply(
            CANcoderConfiguration().with_magnet_sensor(
                MagnetSensorConfigs()
                .with_magnet_offset(encoder_offset)
                .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
            )
        )

    # Tell SysId how to plumb the driving voltage to the motors.
    @typing.override
    def drive(self, voltage: units.volts) -> None:
        self.motor.set_control(VoltageOut(voltage))

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    @typing.override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.motor.get_motor_voltage().value)
            .position(self.absolute_encoder.get_position().value)
            .velocity(self.absolute_encoder.get_velocity().value)
        )

    @typing.override
    def atPositiveLimit(self) -> bool:
        return self.absolute_encoder.get_position().value >= self.positive_limit

    @typing.override
    def atNegativeLimit(self) -> bool:
        return self.absolute_encoder.get_position().value <= self.negative_limit
