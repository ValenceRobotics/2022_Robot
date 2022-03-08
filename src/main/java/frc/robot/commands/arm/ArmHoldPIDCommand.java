// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmHoldPIDCommand extends PIDCommand {
  /** Creates a new ArmHoldPIDCommand. */
  private final ArmSubsystem armSubsystem;
  private final XboxController controller;

  public ArmHoldPIDCommand(XboxController controller, ArmSubsystem armSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(.5, 0, 0),
        // This should return the measurement
        armSubsystem::getArmPosition,
        // This should return the setpoint (can also be a constant)
        armSubsystem.getHoldPos(),
        // This uses the output
        output -> {
          // Use the output here
          armSubsystem.driveArm(output);
          SmartDashboard.putNumber("pid output", output);
          SmartDashboard.putNumber("setpoint", armSubsystem.getHoldPos());
        });
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    this.controller = controller;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(.5);
  }

  @Override
  public void execute() {
      // TODO Auto-generated method stub
      super.execute();
      System.out.println("running pid");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (controller.getLeftTriggerAxis() < .5 && controller.getRightTriggerAxis() < .5);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
  }

  // @Override
  // public void end(boolean interrupted) {
  //     // TODO Auto-generated method stub
  //     super.end(interrupted);
  // }
}
