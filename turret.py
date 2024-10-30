import math
from wpilib import Preferences
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import phoenix6
from phoenix5 import WPI_TalonSRX



class Turret:
    def __init__(self, turningMotorChannel: int, turningEncoderChannel: int) -> None:
        """Constructs a Turret with a turning motor and turning encoder.

        :param turningMotorChannel:    CAN ID for the turning motor.
        :param turningEncoderChannel:  CAN ID for the turning encoder.
        """
        self.turningMotor = WPI_TalonSRX(turningMotorChannel)
        self.turningEncoder = phoenix6.hardware.CANcoder(turningEncoderChannel)

        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            0, 0, 0, wpimath.trajectory.TrapezoidProfile.Constraints(0, 0)
        )
        
        # Feedforward for turret rotation
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 0)
        
        # Initialize preferences for turret control
        self.initPreferences()

        # Enable continuous input for full rotation control
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def initPreferences(self) -> None:
        Preferences.initDouble("turning_kP", 18.0)
        Preferences.initDouble("turning_kS", 0.14)
        Preferences.initDouble("turning_kV", 0.375)
        Preferences.initDouble("turning_max_v", 30.0)
        Preferences.initDouble("turning_max_a", 300.0)

    def collectPreferences(self) -> None:
        # Update PID and feedforward values from preferences
        self.turningPIDController.setP(Preferences.getDouble("turning_kP"))
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            Preferences.getDouble("turning_kS"), Preferences.getDouble("turning_kV")
        )
        self.turningPIDController.setConstraints(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                Preferences.getDouble("turning_max_v"), Preferences.getDouble("turning_max_a")
            )
        )

    def setDesiredAngle(self, desiredAngle: wpimath.geometry.Rotation2d) -> None:
        """Sets the desired angle for the turret.

        :param desiredAngle: Desired angle as a Rotation2d object.
        """
        # Update controllers with new values from networktables
        self.collectPreferences()

        # Get the current angle from the encoder (in radians)
        currentAngle = wpimath.geometry.Rotation2d(
            self.turningEncoder.get_position().value * 2 * math.pi
        )

        # Calculate the turning motor output (in rad/s) from the turning PID controller
        turnFeedback = self.turningPIDController.calculate(
            currentAngle.radians(), desiredAngle.radians()
        )

        # Convert the output to voltage with the feedforward
        turnOutput = self.turnFeedforward.calculate(turnFeedback)

        # Set the motor voltage to reach the desired angle
        self.turningMotor.setVoltage(turnOutput)
