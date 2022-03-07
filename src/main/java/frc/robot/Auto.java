package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Auto {
    public static Command getAutoCommand(DrivetrainSubsystem drivetrain) {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.PathingConstants.ksVolts,
                Constants.PathingConstants.kvVoltSecondsPerMeter,
                Constants.PathingConstants.kaVoltSecondsSquaredPerMeter),
                Constants.PathingConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.PathingConstants.kMaxSpeedMetersPerSecond,
                Constants.PathingConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.PathingConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(Constants.PathingConstants.kRamseteB, Constants.PathingConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
              Constants.PathingConstants.ksVolts,
              Constants.PathingConstants.kvVoltSecondsPerMeter,
              Constants.PathingConstants.kaVoltSecondsSquaredPerMeter),
              Constants.PathingConstants.kDriveKinematics,
              drivetrain::getWheelSpeeds,
            new PIDController(Constants.PathingConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.PathingConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.setPose(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0.0, 0.0));
    }
}
