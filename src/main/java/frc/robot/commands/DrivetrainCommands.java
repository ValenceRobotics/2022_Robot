package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DrivetrainCommands {
    public static Command tankDriveXboxController(DrivetrainSubsystem drivetrain, XboxController controller) {
        return new RunCommand(() -> drivetrain.tankDrive(-controller.getLeftY(), -controller.getRightY()), drivetrain);
    }

    public static Command arcadeDriveXboxController(DrivetrainSubsystem drivetrain, XboxController controller) {
        return new RunCommand(() -> drivetrain.arcadeDrive(-controller.getLeftY(), controller.getRightX()), drivetrain);
    }

    public static Command drivetrainDrive(DrivetrainSubsystem drivetrain, double left, double right) {
        return new InstantCommand(() -> drivetrain.tankDrive(left, right), drivetrain);
    }

    public static Command drivetrainStop(DrivetrainSubsystem drivetrain) {
        return new InstantCommand(() -> drivetrain.tankDrive(0, 0), drivetrain);
    }
}
