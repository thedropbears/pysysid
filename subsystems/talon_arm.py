from typing import override

from commands2 import Command
from commands2.sysid.sysidroutine import SysIdRoutine
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
from wpimath.units import radians, volts

from subsystems.sysid_subsystem import SysidSubsystem

FollowerDescriptor = tuple[TalonFX, bool]


class TalonArm(SysidSubsystem):
    DEPLOY_STEP_VOLTAGE = 1
    DEPLOY_RAMP_RATE = 0.6

    RETRACT_STEP_VOLTAGE = 2
    RETRACT_RAMP_RATE = 1.2

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
                .with_forward_soft_limit_threshold(positive_limit)
                .with_forward_soft_limit_enable(True)
                .with_reverse_soft_limit_threshold(negative_limit)
                .with_reverse_soft_limit_enable(True)
            )
        )

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

        self.absolute_encoder = absolute_encoder
        self.encoder_offset = encoder_offset

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
