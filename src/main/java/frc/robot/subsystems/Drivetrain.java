package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase  {
    private final WPI_TalonSRX m_leftFront = new WPI_TalonSRX(Constants.Drivetrain.kLeftFront);
    private final WPI_TalonSRX m_leftRear = new WPI_TalonSRX(Constants.Drivetrain.kLeftRear);
    private final WPI_TalonSRX m_rightFront = new WPI_TalonSRX(Constants.Drivetrain.kRightFront);
    private final WPI_TalonSRX m_rightRear = new WPI_TalonSRX(Constants.Drivetrain.kRightRear);

    public Drivetrain() {
        m_leftFront.configFactoryDefault();
        m_leftRear.configFactoryDefault();
        m_rightFront.configFactoryDefault();
        m_rightRear.configFactoryDefault();

        m_leftRear.follow(m_leftFront);
        m_rightRear.follow(m_rightFront);

        m_leftFront.setInverted(false);
        m_leftRear.setInverted(false);
        m_rightFront.setInverted(true);
        m_rightRear.setInverted(true);

        m_leftFront.setSensorPhase(true);
        m_rightFront.setSensorPhase(true);
    }

    public void tankDrive(double left, double right) {
        m_leftFront.set(left);
        m_rightFront.set(right);
    }

    public void tankDriveVolts(double left, double right) {
        m_leftFront.setVoltage(left);
        m_rightFront.setVoltage(right);
    }
}
