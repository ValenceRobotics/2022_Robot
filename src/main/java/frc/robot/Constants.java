// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        public static final int kLeftFront = 1;
        public static final int kLeftRear = 3;
        public static final int kRightFront = 0;
        public static final int kRightRear = 2;

       // public static final int[] kLeftEnc = {5,6};
        public static final int kLeftEncA = 0;
        public static final int kLeftEncB = 1;
        public static final int[] kRightEnc = {3,2};

        public static final int kPigeonIMU = 0;
        public static final int kCountsPerRev = 4096;
        public static final double kSensorGearRatio = 1;
        public static final double kGearRatio = 10.71;
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);
        public static final int k100msPerSecond = 10;
        public static final Pose2d kStartPosition = new Pose2d(0, 0, new Rotation2d());
    }

    public static final class PathingConstants {
        public static final int ksVolts = 0;
        public static final int kvVoltSecondsPerMeter = 0;
        public static final int kaVoltSecondsSquaredPerMeter = 0;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.0);
        public static final int kMaxSpeedMetersPerSecond = 0;
        public static final int kMaxAccelerationMetersPerSecondSquared = 0;
        public static final int kPDriveVel = 0;
        public static final double kRamseteB = 0;
        public static final double kRamseteZeta = 0;
    }

    public static final class TimedAuto {
        public static final double kForwardSpeed = 0.2;
        public static final double kBackwardSpeed = -0.3; 
        public static final double kForwardSeconds = 2;
        public static final double kBackwardSeconds = 3;
        public static final double kArmDownSeconds = 3; // TODO: Determine if too long or too short
        public static final double kArmUpViolentSeconds = 0.75; // TODO: Determine if too long or too short
        public static final double kOuttakeSeconds = 1;
    }

    public static final class Arm {
        public static final int kArmMotor = 10;
        public static final double kArmMaxSpeed = 1; // ADJUST THIS
        public static final double kArmBottomPositionEncoderReading = -1; // ADJUST THIS
        public static final double kArmTopPositionEncoderReading = 16; // ADJUST THIS
        public static final int kArmEncoder = 0; // ADJUST THIS
        public static final double kArmEncoderDistance = 0; // ADJUST THIS
        public static final PIDController kArmPIDUp = new PIDController(0.05, 0.02, 0.00); // ADJUST THESE
        public static final PIDController kArmPIDDown = new PIDController(0.04, 0.022, 0.00); // ADJUST THESE
        public static final double kArmUp = 0.2; // ADJUST THIS
        public static final double kArmDown = -0.2; // ADJUST THIS
        public static final double kArmHold = 0; // ADJUST THIS
        public static final double kArmUpViolent = 0.4; 
        public static final int kArmCurrentLimit = 60;
    }

    public static final class Intake {
        public static final int kIntakeMotor = 4;

        public static final double kIntakeSpeed = -0.5;
        public static final double kOutakeSpeed = 1;
    }

    public static final class OI {
        public static final int kXboxController = 1;
        public static final int kGamePadController = 0;
        public static final int kJoystickLeft = 1;
        public static final int kJoystickRight = 0;

        public static final int kArmUpButton = 1;
        public static final int kArmDownButton = 2;

        public static final int kIntakeInButton = 5;
        public static final int kIntakeOutButton = 6;

        public static final int kArmOverrideButton = 4;
        public static final int kArmOverrideDisableButton = 3;

        //public static final int kTankDriveButton = 100;
        //public static final int kArcadeDriveButton = 6;
    }
} 
