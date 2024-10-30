import math
import wpilib
from wpilib import SmartDashboard
from wpilib.simulation import PWMSim
import wpimath.geometry
from turret import Turret  # Assumes turret.py contains the Turret class

class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Initialization of the robot."""
        # Initialize the Turret subsystem with motor and encoder channels
        self.turret = Turret(turningMotorChannel=1, turningEncoderChannel=2)
        
        # Initialize target angle on the SmartDashboard
        SmartDashboard.putNumber("Target Angle (degrees)", 0.0)
        
        # Initialize the Xbox Controller on port 0
        self.controller = wpilib.XboxController(0)

        # Simulate the turning motor for the turret
        self.turningMotorSim = PWMSim(self.turret.turningMotor.getDeviceID())
        
        # Turret rotation state for simulation
        self.simulated_angle_radians = 0.0

    def simulationPeriodic(self) -> None:
        """This function is called periodically during simulation."""
        # Simulate encoder angle based on motor voltage applied
        voltage = self.turningMotorSim.getSpeed()  # Assume this simulates the voltage output
        self.simulated_angle_radians += voltage * 0.01  # Adjust factor for realistic turn speed
        
        # Keep angle within [-pi, pi] for continuous input simulation
        self.simulated_angle_radians = (self.simulated_angle_radians + math.pi) % (2 * math.pi) - math.pi

    def robotPeriodic(self) -> None:
        """This function is called every robot packet, regardless of mode."""
        # Get current turret angle in degrees and send it to SmartDashboard
        current_angle = self.getTurretAngleDegrees()
        SmartDashboard.putNumber("Current Angle (degrees)", current_angle)

    def teleopPeriodic(self) -> None:
        """Called periodically during operator control."""
        # Use the controller's right joystick X-axis to adjust the turret angle
        joystick_x = self.controller.getRightX()
        
        # Convert joystick value to target angle range, e.g., -180 to 180 degrees
        target_angle_degrees = joystick_x * 180.0  # Scale to range [-180, 180]
        
        # Update SmartDashboard with target angle for visualization
        SmartDashboard.putNumber("Target Angle (degrees)", target_angle_degrees)
        
        # Convert target angle to radians and set as desired angle
        target_angle_radians = wpimath.geometry.Rotation2d.fromDegrees(target_angle_degrees)
        self.turret.setDesiredAngle(target_angle_radians)

    def getTurretAngleDegrees(self) -> float:
        """Helper method to get the current turret angle in degrees."""
        # Get the current simulated angle in radians and convert to degrees
        return wpimath.geometry.Rotation2d(self.simulated_angle_radians).degrees()


if __name__ == "__main__":
    wpilib.run(Robot)
