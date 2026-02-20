import typing

from phoenix6.configs import (
    FeedbackConfigs,
    MotorOutputConfigs,
    TalonFXConfiguration,
)
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig
from wpilib import sysid
from wpimath.units import volts

from subsystems.sysid_subsystem import SysidSubsystem


class Climber(SysidSubsystem):

    def __init__(
        self,
        motor: TalonFX,
        sensor_to_mechanism_gearing: float,
        limit_switch_host: SparkMax,  

    ) -> None:
        super().__init__(ramp_rate=0.5, step_voltage=1.75, timeout=10.0)

        self.motor = motor

        self.forward_limit_switch = limit_switch_host.getForwardLimitSwitch()
        self.reverse_limit_switch = limit_switch_host.getReverseLimitSwitch()

        sensor_config = SparkMaxConfig()
        sensor_config.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyClosed
        ).forwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotor
        ).reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyClosed
        ).reverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
        ).reverseLimitSwitchPosition(0)

        motor_output_config = MotorOutputConfigs().with_neutral_mode(
            NeutralModeValue.BRAKE
        )

        feedback_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            sensor_to_mechanism_gearing
        )

        motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(motor_output_config)
            .with_feedback(feedback_config)
        )

        self._voltage_req = VoltageOut(0.0)

    def at_forward_limit(self) -> bool:
        return self.forward_limit_switch.get()

    def at_reverse_limit(self) -> bool:
        return self.reverse_limit_switch.get()

    # Tell SysId how to plumb the driving voltage to the motors.
    @typing.override
    def drive(self, voltage: volts) -> None:
        self.motor.set_control(VoltageOut(voltage))

    # Tell SysId how to record a frame of data for the mechanism being characterized.
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
        return not self.at_forward_limit()

    @typing.override
    def beforeNegativeLimit(self) -> bool:
        return not self.at_reverse_limit()
