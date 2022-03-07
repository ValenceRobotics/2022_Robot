// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DrivetrainCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.arm.ArmHoldPIDCommand;
import frc.robot.commands.arm.DriveArmCommand;
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
  public static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
  public static final ArmSubsystem m_arm = new ArmSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();

  public static final XboxController m_xboxController = new XboxController(Constants.OI.kXboxController);
  

  public RobotContainer() {
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(DrivetrainCommands.tankDriveXboxController(m_drivetrain, m_xboxController));
    m_arm.setDefaultCommand(new DriveArmCommand(m_xboxController, m_arm));
  }

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
    //  .whenPressed(DrivetrainCommands.tankDriveXboxController(m_drivetrain, m_xboxController));
    // (new JoystickButton(m_xboxController, Constants.OI.kArcadeDriveButton))
    //  .whenPressed(DrivetrainCommands.arcadeDriveXboxController(m_drivetrain, m_xboxController));
  }

  public Command getAutonomousCommand() {
    return Auto.getAutoCommand(m_drivetrain);
  }
}
