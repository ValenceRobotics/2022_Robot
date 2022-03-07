// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmHoldPIDCommand extends PIDCommand {
  /** Creates a new ArmHoldPIDCommand. */
  public ArmHoldPIDCommand(ArmSubsystem armSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        armSubsystem::getArmPosition,
        // This should return the setpoint (can also be a constant)
        armSubsystem.holdPosition,
        // This uses the output
        output -> {
          // Use the output here
          armSubsystem.driveArm(output);
        });
    addRequirements(armSubsystem);
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
    return getController().atSetpoint();
  }
}
