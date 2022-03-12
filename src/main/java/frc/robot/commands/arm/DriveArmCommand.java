// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class DriveArmCommand extends CommandBase {
  /** Creates a new DriveArmCommand. */
  private final ArmSubsystem armSubsystem; 
  private final XboxController controller;

  public DriveArmCommand(XboxController controller, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.controller = controller;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getYButton()) {
      armSubsystem.setHoldPos();
    }
    if (controller.getLeftTriggerAxis() >= 0.5) {
          armSubsystem.driveArm(Constants.Arm.kArmDown);
          // System.out.println("arm up");
      } else if (controller.getRightTriggerAxis() >= 0.5) {
          armSubsystem.driveArm(Constants.Arm.kArmUp);
          // System.out.println("arm down");
      } else {
          //armHoldCommandPID.schedule();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // armSubsystem.driveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}