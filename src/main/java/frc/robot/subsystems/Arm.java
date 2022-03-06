package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.kArmMotor, MotorType.kBrushless);

    public Arm() {
        m_armMotor.restoreFactoryDefaults();

        // Set this accordingly depending the lights on the speed controller
        // m_armMotor.setInverted(true);
    }

    public void driveArm(double speed) {
        m_armMotor.set(speed);
    }
}
