package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands {
    public static Command intakeIn(IntakeSubsystem intake) {
        return new InstantCommand(() -> intake.driveIntake(Constants.Intake.kIntakeSpeed), intake);
    }

    public static Command intakeOut(IntakeSubsystem intake) {
        return new InstantCommand(() -> intake.driveIntake(-Constants.Intake.kIntakeSpeed), intake);
    }

    public static Command intakeStop(IntakeSubsystem intake) {
        return new InstantCommand(() -> intake.driveIntake(0), intake);
    }
}
