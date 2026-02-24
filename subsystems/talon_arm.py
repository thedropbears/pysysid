from typing import override

from phoenix6.configs import (
    FeedbackConfigs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.controls import Follower, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, MotorAlignmentValue, NeutralModeValue
from wpilib import DutyCycleEncoder, sysid
from wpimath.units import radians, radiansToRotations, volts

from subsystems.sysid_subsystem import SysidSubsystem

FollowerDescriptor = tuple[TalonFX, bool]


class TalonArm(SysidSubsystem):
    def __init__(
        self,
        arm_motor: TalonFX,
        *followers: FollowerDescriptor,
        invert_motor: bool,
        motor_to_mechanism_gearing: float,
        absolute_encoder: DutyCycleEncoder,
        encoder_offset: float,
        positive_limit: radians,
        negative_limit: radians,
    ):
        super().__init__()

        motor_output_configs = (
            MotorOutputConfigs()
            .with_inverted(
                InvertedValue.CLOCKWISE_POSITIVE
                if invert_motor
                else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
            .with_neutral_mode(NeutralModeValue.BRAKE)
        )
        feedback_configs = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            motor_to_mechanism_gearing
        )

        self.arm_motor = arm_motor
        arm_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(motor_output_configs)
            .with_feedback(feedback_configs)
            .with_software_limit_switch(
                SoftwareLimitSwitchConfigs()
                .with_forward_soft_limit_threshold(radiansToRotations(positive_limit))
                .with_forward_soft_limit_enable(True)
                .with_reverse_soft_limit_threshold(radiansToRotations(negative_limit))
                .with_reverse_soft_limit_enable(True)
            )
        )
        arm_motor.set_position(absolute_encoder.get() + encoder_offset)

        self.followers = [motor for motor, _ in followers]

        for motor, is_opposed in followers:
            motor.configurator.apply(
                TalonFXConfiguration()
                .with_feedback(feedback_configs)
                .with_motor_output(motor_output_configs)
            )
            motor.set_control(
                Follower(
                    arm_motor.device_id,
                    (
                        MotorAlignmentValue.OPPOSED
                        if is_opposed
                        else MotorAlignmentValue.ALIGNED
                    ),
                )
            )

    @override
    def drive(self, voltage: volts) -> None:
        self.arm_motor.set_control(VoltageOut(voltage))

    @override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.arm_motor.get_motor_voltage().value)
            .position(self.arm_motor.get_position().value)
            .velocity(self.arm_motor.get_velocity().value)
        )
        for motor in self.followers:
            (
                sys_id_routine.motor(f"follower-{motor.device_id}")
                .voltage(motor.get_motor_voltage().value)
                .position(motor.get_position().value)
                .velocity(motor.get_velocity().value)
            )

    @override
    def beforePositiveLimit(self) -> bool:
        return not self.arm_motor.get_fault_forward_soft_limit().value

    @override
    def beforeNegativeLimit(self) -> bool:
        return not self.arm_motor.get_fault_reverse_soft_limit().value
