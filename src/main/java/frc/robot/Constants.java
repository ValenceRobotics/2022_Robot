// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
        public static final int kLeftFront = 0;
        public static final int kLeftRear = 1;
        public static final int kRightFront = 2;
        public static final int kRightRear = 3;
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

    public static final class OI {
        public static final int kXboxController = 0;
    }
}
