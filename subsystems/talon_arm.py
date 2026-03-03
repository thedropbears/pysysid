import math
from typing import override

from commands2 import Command
from commands2.sysid.sysidroutine import SysIdRoutine
from phoenix6.configs import (
    CANcoderConfiguration,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.controls import Follower, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    InvertedValue,
    MotorAlignmentValue,
    NeutralModeValue,
    SensorDirectionValue,
)
from wpilib import sysid
from wpimath.units import radians, turns, volts

from subsystems.sysid_subsystem import SysidSubsystem

FollowerDescriptor = tuple[TalonFX, bool]


class TalonArm(SysidSubsystem):
    DEPLOY_STEP_VOLTAGE = 1
    DEPLOY_RAMP_RATE = 0.6

    RETRACT_STEP_VOLTAGE = 2.0
    RETRACT_RAMP_RATE = 1.75

    def __init__(
        self,
        arm_motor: TalonFX,
        *followers: FollowerDescriptor,
        invert_motor: bool,
        motor_to_encoder_gearing: float,
        encoder_to_mechanism_gearing: float,
        absolute_encoder: CANcoder,
        encoder_offset: turns,
        positive_limit: radians,
        negative_limit: radians,
    ):
        super().__init__(
            step_voltage=self.DEPLOY_STEP_VOLTAGE, ramp_rate=self.DEPLOY_RAMP_RATE
        )

        motor_output_configs = (
            MotorOutputConfigs()
            .with_inverted(
                InvertedValue.CLOCKWISE_POSITIVE
                if invert_motor
                else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
            .with_neutral_mode(NeutralModeValue.BRAKE)
        )

        feedback_configs = (
            FeedbackConfigs()
            .with_sensor_to_mechanism_ratio(encoder_to_mechanism_gearing)
            .with_rotor_to_sensor_ratio(motor_to_encoder_gearing)
            .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
            .with_feedback_remote_sensor_id(absolute_encoder.device_id)
        )

        self.arm_motor = arm_motor
        arm_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(motor_output_configs)
            .with_feedback(feedback_configs)
            .with_software_limit_switch(
                SoftwareLimitSwitchConfigs()
                .with_forward_soft_limit_threshold(positive_limit / math.tau)
                .with_forward_soft_limit_enable(True)
                .with_reverse_soft_limit_threshold(negative_limit / math.tau)
                .with_reverse_soft_limit_enable(True)
            )
        )

        self.positive_limit = (positive_limit - math.radians(20)) / math.tau
        self.negative_limit = (negative_limit + math.radians(5)) / math.tau

        self.followers = followers

        for motor, _ in self.followers:
            motor.configurator.apply(
                TalonFXConfiguration()
                .with_feedback(feedback_configs)
                .with_motor_output(motor_output_configs)
            )

        encoder_magnet_config = (
            MagnetSensorConfigs()
            .with_magnet_offset(encoder_offset)
            .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
        )

        absolute_encoder.configurator.apply(
            CANcoderConfiguration().with_magnet_sensor(encoder_magnet_config)
        )

        self.absolute_encoder = absolute_encoder

    @override
    def drive(self, voltage: volts) -> None:
        self.arm_motor.set_control(VoltageOut(voltage))
        for motor, is_opposed in self.followers:
            motor.set_control(
                Follower(
                    self.arm_motor.device_id,
                    (
                        MotorAlignmentValue.OPPOSED
                        if is_opposed
                        else MotorAlignmentValue.ALIGNED
                    ),
                )
            )

    @override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.arm_motor.get_motor_voltage().value)
            .position(self.absolute_encoder.get_position().value)
            .velocity(self.absolute_encoder.get_velocity().value)
        )

    @override
    def beforePositiveLimit(self) -> bool:
        return self.absolute_encoder.get_position().value < self.positive_limit

    @override
    def beforeNegativeLimit(self) -> bool:
        return self.absolute_encoder.get_position().value > self.negative_limit

    @override
    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        if direction == SysIdRoutine.Direction.kForward:
            self.setRampRate(self.RETRACT_RAMP_RATE)
        else:
            self.setRampRate(self.DEPLOY_RAMP_RATE)

        return super().sysIdQuasistatic(direction)

    @override
    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        if direction == SysIdRoutine.Direction.kForward:
            self.setStepVoltage(self.RETRACT_STEP_VOLTAGE)
        else:
            self.setStepVoltage(self.DEPLOY_STEP_VOLTAGE)

        return super().sysIdDynamic(direction)
