#!/usr/bin/env python3

import phoenix5
import wpilib
import math


class SwerveDriveRobot(wpilib.TimedRobot):
    kSlotIdx = 0
    kPIDLoopIdx = 0
    kTimeoutMs = 10

    def robotInit(self):
        # Drive motors
        self.drive_motors = [
            phoenix5.WPI_TalonFX(i) for i in range(1, 5)  # Falcon 500 motors
        ]

        # Turning motors
        self.turning_motors = [
            phoenix5.WPI_TalonSRX(i + 10) for i in range(1, 5)  # Talon SRX for turning
        ]

        # Joystick for control
        self.joystick = wpilib.Joystick(0)

        # Initialize turning motors with Lamprey 2 absolute encoders
        for i, motor in enumerate(self.turning_motors):
            # Configure feedback sensor
            motor.configSelectedFeedbackSensor(
                phoenix5.FeedbackDevice.CTRE_MagEncoder_Absolute,
                self.kPIDLoopIdx,
                self.kTimeoutMs,
            )

            # Sensor phase to ensure correct direction
            motor.setSensorPhase(True)
            motor.setInverted(False)

            # Configure output settings
            motor.configNominalOutputForward(0, self.kTimeoutMs)
            motor.configNominalOutputReverse(0, self.kTimeoutMs)
            motor.configPeakOutputForward(1, self.kTimeoutMs)
            motor.configPeakOutputReverse(-1, self.kTimeoutMs)

            # Set PID values for turning motors
            motor.configAllowableClosedloopError(0, self.kPIDLoopIdx, self.kTimeoutMs)
            motor.selectProfileSlot(self.kSlotIdx, self.kPIDLoopIdx)
            motor.config_kF(0, 0, self.kTimeoutMs)
            motor.config_kP(0, 0.1, self.kTimeoutMs)
            motor.config_kI(0, 0, self.kTimeoutMs)
            motor.config_kD(0, 0, self.kTimeoutMs)

            
    def calculateSwerveOutputs(self, x, y, rotation):
        """
        Calculate the drive and turn outputs for each swerve module
        """
        # Robot dimensions for wheelbase and track width
        wheelbase = 0.5  # Meters
        track_width = 0.5  # Meters
        r = math.sqrt((wheelbase / 2) ** 2 + (track_width / 2) ** 2)

        # Calculate speed and angle for each wheel
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

        speeds, angles = self.calculateSwerveOutputs(x, y, rotation)

        for i in range(4):
            # Set drive motor speed
            self.drive_motors[i].set(phoenix5.ControlMode.PercentOutput, speeds[i])

            # Set turning motor to target position
            target_position = angles[i] * (4096 / 360.0)  # Convert degrees to encoder units
            self.turning_motors[i].set(phoenix5.ControlMode.Position, target_position)


if __name__ == "__main__":
    wpilib.run(SwerveDriveRobot)
