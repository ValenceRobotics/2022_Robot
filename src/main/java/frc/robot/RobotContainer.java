// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DrivetrainCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  public static final ArmSubsystem m_arm = new ArmSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();

  public static final XboxController m_xboxController = new XboxController(Constants.OI.kXboxController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(DrivetrainCommands.tankDriveXboxController(m_drivetrain, m_xboxController));
    m_arm.setDefaultCommand(ArmCommands.armTriggerOperation(m_arm, m_xboxController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    (new JoystickButton(m_xboxController, Constants.OI.kIntakeInButton))
      .whileHeld(IntakeCommands.intakeIn(m_intake))
      .whenReleased(IntakeCommands.intakeStop(m_intake));

    (new JoystickButton(m_xboxController, Constants.OI.kIntakeOutButton))
      .whileHeld(IntakeCommands.intakeOut(m_intake))
      .whenReleased(IntakeCommands.intakeStop(m_intake));

    // For use when not using triggers to operate the arm
    // (new JoystickButton(m_xboxController, Constants.OI.kArmUpButton))
    //   .whenPressed(ArmCommands.armUp(m_arm));

    // (new JoystickButton(m_xboxController, Constants.OI.kArmDownButton))
    //   .whenPressed(ArmCommands.armDown(m_arm));

    // Uncomment this when you want buttons to switch between tank and arcade drive
    // (new JoystickButton(m_xboxController, Constants.OI.kTankDriveButton))
      // .whenPressed(DrivetrainCommands.tankDriveXboxController(m_drivetrain, m_xboxController));
    // (new JoystickButton(m_xboxController, Constants.OI.kArcadeDriveButton))
      // .whenPressed(DrivetrainCommands.arcadeDriveXboxController(m_drivetrain, m_xboxController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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
            m_drivetrain::getPose,
            new RamseteController(Constants.PathingConstants.kRamseteB, Constants.PathingConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
              Constants.PathingConstants.ksVolts,
              Constants.PathingConstants.kvVoltSecondsPerMeter,
              Constants.PathingConstants.kaVoltSecondsSquaredPerMeter),
              Constants.PathingConstants.kDriveKinematics,
              m_drivetrain::getWheelSpeeds,
            new PIDController(Constants.PathingConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.PathingConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.setPose(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0.0, 0.0));
  }
}
