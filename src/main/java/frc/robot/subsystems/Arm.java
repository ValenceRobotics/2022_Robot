package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final WPI_VictorSPX m_armMotor = new WPI_VictorSPX(Constants.Arm.kArmMotor);

    public Arm() {
        m_armMotor.configFactoryDefault();

        // Set this accordingly depending the lights on the speed controller
        // m_armMotor.setInverted(true);
    }

    public void driveArm(double speed) {
        m_armMotor.set(speed);
    }
}
