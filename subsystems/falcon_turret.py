import typing

from phoenix6.configs import (
    ExternalFeedbackConfigs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    TalonFXSConfiguration,
)
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFXS
from phoenix6.signals import NeutralModeValue
from wpilib import sysid
from wpimath.units import degrees, degreesToRotations, volts

from subsystems.sysid_subsystem import SysidSubsystem


class FalconTurret(SysidSubsystem):
    def __init__(
        self,
        motor: TalonFXS,
        sensor_to_mechanism_gearing: float,
        positive_limit: degrees,
        negative_limit: degrees,
    ) -> None:
        super().__init__(ramp_rate=0.5, step_voltage=1.75, timeout=10.0)

        self.motor = motor
        self.positive_limit = positive_limit
        self.negative_limit = negative_limit

        motor_output_config = MotorOutputConfigs().with_neutral_mode(
            NeutralModeValue.BRAKE
        )
        external_feedback_config = (
            ExternalFeedbackConfigs().with_sensor_to_mechanism_ratio(
                sensor_to_mechanism_gearing
            )
        )

        limit_switch_config = (
            SoftwareLimitSwitchConfigs()
            .with_forward_soft_limit_threshold(degreesToRotations(positive_limit))
            .with_forward_soft_limit_enable(True)
            .with_reverse_soft_limit_threshold(degreesToRotations(negative_limit))
            .with_reverse_soft_limit_enable(True)
        )

        motor.configurator.apply(
            TalonFXSConfiguration()
            .with_motor_output(motor_output_config)
            .with_external_feedback(external_feedback_config)
            .with_software_limit_switch(limit_switch_config)
        )

    # Tell SysId how to plumb the driving voltage to the motors.
    @typing.override
    def drive(self, voltage: volts) -> None:
        self.motor.set_control(VoltageOut(voltage))

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    @typing.override
    def log(self, sys_id_routine: sysid.SysIdRoutineLog) -> None:
        (
            sys_id_routine.motor("leader")
            .voltage(self.motor.get_motor_voltage().value)
            .position(self.motor.get_position().value)
            .velocity(self.motor.get_velocity().value)
        )

    @typing.override
    def beforePositiveLimit(self) -> bool:
        return self.motor.get_position().value < self.positive_limit

    @typing.override
    def beforeNegativeLimit(self) -> bool:
        return self.motor.get_position().value > self.negative_limit
