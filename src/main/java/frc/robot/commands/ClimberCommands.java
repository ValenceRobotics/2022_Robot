package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommands {
    public static Command climberLowerSlow(ClimberSubsystem climber, Boolean left) {
        if (left)
            return new InstantCommand(() -> climber.setSpeedLeft(-Constants.Climber.kdownIndvSpd), climber);
        else
            return new InstantCommand(() -> climber.setSpeedRight(-Constants.Climber.kdownIndvSpd), climber);
    }

    public static Command climberStop(ClimberSubsystem climber, Boolean left) {
        if (left)
            return new InstantCommand(() -> climber.setSpeedLeft(0), climber);
        else
            return new InstantCommand(() -> climber.setSpeedRight(0), climber);
    }
}
