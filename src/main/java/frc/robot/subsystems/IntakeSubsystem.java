package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(Constants.Intake.kIntakeMotor);

    public IntakeSubsystem() {
        m_intakeMotor.configFactoryDefault();

        // Set this accordingly depending the lights on the speed controller
        // m_armMotor.setInverted(true);
    }

    public void driveIntake(double speed) {
        m_intakeMotor.set(speed);
    }
}
