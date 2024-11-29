import phoenix6
import phoenix5
import wpilib
import math
from wpilib.interfaces import Gyro
from networktables import NetworkTablesInstance


class SwerveDriveRobot(wpilib.TimedRobot):
    kSlotIdx = 0
    kPIDLoopIdx = 0
    kTimeoutMs = 10

    def robotInit(self):
        # Drive motors
        self.drive_motors = [
            phoenix6.hardware.TalonFX(i) for i in range(1, 5)  # Falcon 500 motors
        ]

        # Turning motors
        self.turning_motors = [
            phoenix5.WPI_TalonSRX(i + 10) for i in range(1, 5)  # Talon SRX for turning
        ]

        # Joystick for control
        self.joystick = wpilib.Joystick(0)

        # Gyro for field-relative control
        self.gyro = wpilib.ADXRS450_Gyro(wpilib.SPI.Port.kOnboardCS0)
        self.gyro.calibrate()

        # Initialize turning motors with Lamprey 2 absolute encoders
        for i, motor in enumerate(self.turning_motors):
            motor.configSelectedFeedbackSensor(
                phoenix5.FeedbackDevice.CTRE_MagEncoder_Absolute,
                self.kPIDLoopIdx,
                self.kTimeoutMs,
            )
            motor.setSensorPhase(True)
            motor.setInverted(False)
            motor.configNominalOutputForward(0, self.kTimeoutMs)
            motor.configNominalOutputReverse(0, self.kTimeoutMs)
            motor.configPeakOutputForward(1, self.kTimeoutMs)
            motor.configPeakOutputReverse(-1, self.kTimeoutMs)
            motor.configAllowableClosedloopError(0, self.kPIDLoopIdx, self.kTimeoutMs)
            motor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
            motor.config_kF(0, 0, self.kTimeoutMs)
            motor.config_kP(0, 0.1, self.kTimeoutMs)
            motor.config_kI(0, 0, self.kTimeoutMs)
            motor.config_kD(0, 0, self.kTimeoutMs)

        # NetworkTables for AdvantageScope
        self.nt_instance = NetworkTablesInstance.getDefault()
        self.nt_instance.startServer()
        self.swerve_publisher = self.nt_instance.getTable("SmartDashboard").getEntry("SwerveStates")

    def calculateSwerveOutputs(self, x, y, rotation):
        """
        Calculate the drive and turn outputs for each swerve module
        """
        wheelbase = 0.5  # Meters
        track_width = 0.5  # Meters
        r = math.sqrt((wheelbase / 2) ** 2 + (track_width / 2) ** 2)

        a = x - rotation * (wheelbase / r)
        b = x + rotation * (wheelbase / r)
        c = y - rotation * (track_width / r)
        d = y + rotation * (track_width / r)

        speeds = [
            math.sqrt(b**2 + d**2),
            math.sqrt(b**2 + c**2),
            math.sqrt(a**2 + d**2),
            math.sqrt(a**2 + c**2),
        ]

        angles = [
            math.degrees(math.atan2(b, d)),
            math.degrees(math.atan2(b, c)),
            math.degrees(math.atan2(a, d)),
            math.degrees(math.atan2(a, c)),
        ]

        max_speed = max(speeds)
        if max_speed > 1:
            speeds = [s / max_speed for s in speeds]

        return speeds, angles

    def teleopPeriodic(self):
        x = self.joystick.getX()  # Lateral movement
        y = -self.joystick.getY()  # Forward movement
        rotation = self.joystick.getZ()  # Rotation

        # Get gyro angle in radians
        gyro_angle = math.radians(self.gyro.getAngle())

        # Field-relative transformations
        temp_x = x * math.cos(gyro_angle) - y * math.sin(gyro_angle)
        temp_y = x * math.sin(gyro_angle) + y * math.cos(gyro_angle)
        x, y = temp_x, temp_y

        speeds, angles = self.calculateSwerveOutputs(x, y, rotation)

        for i in range(4):
            # Set drive motor speed
            self.drive_motors[i].set_control(speeds[i])

            # Set turning motor to target position
            target_position = angles[i] * (4096 / 360.0)  # Convert degrees to encoder units
            self.turning_motors[i].set(phoenix5.ControlMode.Position, target_position)

            # Publish individual module states
            self.nt_instance.getTable("AdvantageScope").getEntry(f"Module{i+1}_Speed").setValue(speeds[i])
            self.nt_instance.getTable("AdvantageScope").getEntry(f"Module{i+1}_Angle").setValue(angles[i])



if __name__ == "__main__":
    wpilib.run(SwerveDriveRobot)
