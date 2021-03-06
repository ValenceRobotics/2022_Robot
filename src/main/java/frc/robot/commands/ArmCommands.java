package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommands {
    public static Command armUp(ArmSubsystem arm) {
        return new InstantCommand(() -> arm.driveArm(Constants.Arm.kArmUp), arm);
    }

    public static Command armDown(ArmSubsystem arm) {
        double armPower = SmartDashboard.getNumber("armPowerDown", Constants.Arm.kArmDown);
        return new InstantCommand(() -> arm.driveArm(armPower), arm);
    }

    public static Command armUpViolent(ArmSubsystem arm) {
        return new InstantCommand(() -> arm.driveArm(Constants.Arm.kArmUpViolent), arm);
    }

    // public static Command armTriggerOperation(ArmSubsystem arm, XboxController controller) {
        // return new RunCommand(() -> {
            //if (controller.getLeftTriggerAxis() > 0.5) {
                // arm.driveArm(Constants.Arm.kArmUp);
           // } else if (controller.getRightTriggerAxis() > 0.5) {
                // arm.driveArm(Constants.Arm.kArmDown);
            // } else {
                
            // }
        // }, arm);
    // }

    public static Command armPidUp(ArmSubsystem arm) {
        return new PIDCommand(Constants.Arm.kArmPIDUp, arm::getArmPosition, Constants.Arm.kArmTopPositionEncoderReading, arm::driveArm, arm);
    }

    public static Command armPidDown(ArmSubsystem arm) {
        return new PIDCommand(Constants.Arm.kArmPIDDown, arm::getArmPosition, Constants.Arm.kArmBottomPositionEncoderReading, arm::driveArm, arm);
    }

    public static Command armResetEncoder(ArmSubsystem arm) {
        return new InstantCommand(() -> arm.resetArmEncoder());
    }
}
