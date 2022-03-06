package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.kArmMotor, MotorType.kBrushless);
    private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(Constants.Arm.kArmEncoder);

    public Arm() {
        // m_armMotor.configFactoryDefault();
        m_armMotor.setInverted(false);
        // Set this accordingly depending the lights on the speed controller
        // m_armMotor.setInverted(true);

        m_armEncoder.reset();
        m_armEncoder.setDistancePerRotation(Constants.Arm.kArmEncoderDistance);
    }

    public double getArmPosition() {
        return m_armEncoder.getDistance();
    }

    public void driveArm(double speed) {
        m_armMotor.set(MathUtil.clamp(speed, -Constants.Arm.kArmMaxSpeed, Constants.Arm.kArmMaxSpeed));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", getArmPosition());
    }
}
