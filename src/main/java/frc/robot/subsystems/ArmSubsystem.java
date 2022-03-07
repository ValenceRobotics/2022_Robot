package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.kArmMotor, MotorType.kBrushless);
    private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    public double holdPosition;

    public ArmSubsystem() {
        // m_armMotor.configFactoryDefault();
        m_armMotor.setInverted(false);
        // Set this accordingly depending the lights on the speed controller
        // m_armMotor.setInverted(true);

        m_armEncoder.setPosition(0);
        m_armEncoder.setPositionConversionFactor(Constants.Arm.kArmEncoderDistance); //multiply by factor to
    }

    public double getArmPosition() { //can this return the height of intake
        return m_armEncoder.getPosition(); //returns % of full arm rotation
    }

    public double getArmAngle(){ //can this return the angle of rotation? gear ration is 80:1 (80 turns of motor for 1 full rotation of arm)
        return 0.0;
    }

    public void driveArm(double speed) {
        m_armMotor.set(MathUtil.clamp(speed, -Constants.Arm.kArmMaxSpeed, Constants.Arm.kArmMaxSpeed));
        holdPosition = getArmPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        System.out.println("Arm position " + getArmPosition());
    }
}
