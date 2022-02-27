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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();

  private final XboxController m_xboxController = new XboxController(Constants.OI.kXboxController);
  // private final Joystick m_joystickLeft = new Joystick(Constants.OI.kJoystickLeft);
  // private final Joystick m_joystickRight = new Joystick(Constants.OI.kJoystickRight);

  private final Command m_tankDrive = new RunCommand(() -> m_drivetrain.tankDrive(m_xboxController.getLeftY(), m_xboxController.getRightY()), m_drivetrain);
  // private final Command m_tankDrive = new RunCommand(() -> m_drivetrain.tankDrive(m_joystickLeft.getY(), m_joystickRight.getY()), m_drivetrain);

  // private final Button m_armUp = new JoystickButton(m_joystickRight, Constants.OI.kArmUpButton);
  // private final Button m_armDown = new JoystickButton(m_joystickRight, Constants.OI.kArmDownButton);
  // private final Button m_intakeIn = new JoystickButton(m_joystickRight, Constants.OI.kIntakeInButton);
  // private final Button m_intakeOut = new JoystickButton(m_joystickRight, Constants.OI.kIntakeOutButton);
  private final Button m_armUp = new JoystickButton(m_xboxController, Constants.OI.kArmUpButton);
  private final Button m_armDown = new JoystickButton(m_xboxController, Constants.OI.kArmDownButton);
  private final Button m_intakeIn = new JoystickButton(m_xboxController, Constants.OI.kIntakeInButton);
  private final Button m_intakeOut = new JoystickButton(m_xboxController, Constants.OI.kIntakeOutButton);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(m_tankDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_armUp.whileHeld(new RunCommand(() -> m_arm.driveArm(Constants.Arm.kArmSpeed), m_arm));
    m_armDown.whileHeld(new RunCommand(() -> m_arm.driveArm(-Constants.Arm.kArmSpeed), m_arm));

    m_intakeIn.whileHeld(new RunCommand(() -> m_intake.driveIntake(Constants.Intake.kIntakeSpeed), m_intake));
    m_intakeOut.whileHeld(new RunCommand(() -> m_intake.driveIntake(-Constants.Intake.kIntakeSpeed), m_intake));
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
