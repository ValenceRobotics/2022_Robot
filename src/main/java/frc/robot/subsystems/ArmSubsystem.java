package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.kArmMotor, MotorType.kBrushless);
    private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    private double holdPosition;

    private PIDState m_pidState = PIDState.Disabled;

    public enum PIDState {
        Up,
        Down,
        Disabled
    }

    public ArmSubsystem() {
        // m_armMotor.configFactoryDefault();
        m_armMotor.setInverted(false);
        m_armMotor.setSmartCurrentLimit(Constants.Arm.kArmCurrentLimit);
        // Set this accordingly depending the lights on the speed controller
        // m_armMotor.setInverted(true);
        m_armMotor.setSoftLimit(SoftLimitDirection.kForward, 9999999);
        m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, -9999999);
        m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

        m_armEncoder.setPosition(0);
        m_armEncoder.setPositionConversionFactor(Constants.Arm.kArmEncoderDistance); //multiply by factor to
    }

    public void setSoftLimit(){
    //     m_armMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Arm.forwardSoftLimit);
    //     m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.reverseSoftLimit);
    }

    public double getArmPosition() { //can this return the height of intake
        return m_armEncoder.getPosition(); //returns % of full arm rotation
    }

    public double getArmAngle(){ //can this return the angle of rotation? gear ration is 80:1 (80 turns of motor for 1 full rotation of arm)
        return 0.0;
    }

    public void resetArmEncoder() {
        m_armEncoder.setPosition(0);
    }

    public void driveArm(double speed) {
        m_armMotor.set(MathUtil.clamp(speed, -Constants.Arm.kArmMaxSpeed, Constants.Arm.kArmMaxSpeed));
        // m_armMotor.set(0);
    }

    public double getHoldPos(){
        return holdPosition;
    }

    public void setHoldPos(){
        holdPosition = getArmPosition();
    }   

    public void setPidState(PIDState state) {
        m_pidState = state;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Arm Position", getArmPosition());
        // SmartDashboard.putNumber("Arm Power", m_armMotor.get());

        // System.out.println("encorder amr: " + m_armEncoder.getPosition());

        if (m_pidState == PIDState.Up)
            driveArm(Constants.Arm.kArmPIDUp.calculate(getArmPosition(), Constants.Arm.kArmTopPositionEncoderReading));
        else if (m_pidState == PIDState.Down)
            driveArm(Constants.Arm.kArmPIDDown.calculate(getArmPosition(), Constants.Arm.kArmBottomPositionEncoderReading));
        // else if (m_pidState == PIDState.Disabled)
        //     driveArm(0);

        // if (getArmPosition() < 1 && m_armMotor.get() < 0.05)
        //     resetArmEncoder();
    }
}
