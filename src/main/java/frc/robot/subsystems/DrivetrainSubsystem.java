package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase  {
    private final WPI_TalonSRX m_leftFront = new WPI_TalonSRX(Constants.Drivetrain.kLeftFront);
    private final WPI_VictorSPX m_leftRear = new WPI_VictorSPX(Constants.Drivetrain.kLeftRear);
    private final WPI_TalonSRX m_rightFront = new WPI_TalonSRX(Constants.Drivetrain.kRightFront);
    private final WPI_VictorSPX m_rightRear = new WPI_VictorSPX(Constants.Drivetrain.kRightRear);

    private final WPI_Pigeon2 m_imu = new WPI_Pigeon2(Constants.Drivetrain.kPigeonIMU);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_imu.getRotation2d(), Constants.Drivetrain.kStartPosition);
    private final Field2d m_field = new Field2d();

    public DrivetrainSubsystem() {
        // reset all motors
        m_leftFront.configFactoryDefault();
        m_leftRear.configFactoryDefault();
        m_rightFront.configFactoryDefault();
        m_rightRear.configFactoryDefault();

        // set follow for rear motors
        m_leftRear.follow(m_leftFront);
        m_rightRear.follow(m_rightFront);

        // Setup motor inversion
        m_rightFront.setInverted(true);
        m_leftFront.setInverted(false);
        m_rightRear.setInverted(InvertType.FollowMaster);
        m_leftRear.setInverted(InvertType.FollowMaster);

        m_leftFront.setSensorPhase(true);
        m_rightFront.setSensorPhase(true);

        m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        resetEncoders();

        //m_imu.configFactoryDefault();
        //m_imu.configMountPose(-180, 0, 90);

        SmartDashboard.putData("Field", m_field);
    }

    public void tankDrive(double left, double right) {
        m_leftFront.set(left);
        m_rightFront.set(right);
    }

    public void tankDriveVolts(double left, double right) {
        m_leftFront.setVoltage(left);
        m_rightFront.setVoltage(right);
    }

    public void arcadeDrive(double throttle, double turn) {
        turn *= 0.4;
        m_leftFront.set(throttle + turn);
        m_rightFront.set(throttle - turn);
    }

    @Override
    public void periodic() {
        m_odometry.update(m_imu.getRotation2d(), quadratureUnitsToMeters(m_leftFront.getSelectedSensorPosition()), quadratureUnitsToMeters(m_rightFront.getSelectedSensorPosition()));
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("encoder left", m_leftFront.getSelectedSensorPosition());
        SmartDashboard.putNumber("encoder right", m_rightFront.getSelectedSensorPosition());

        // For debug
        System.out.println("Left: " + m_leftFront.getSelectedSensorPosition());
        System.out.println("Right: " + m_rightFront.getSelectedSensorPosition());
    }

    private void resetEncoders() {
        m_leftFront.setSelectedSensorPosition(0);
        m_rightFront.setSelectedSensorPosition(0);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_imu.getRotation2d());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(quadratureUnitsToMeters(m_leftFront.getSelectedSensorVelocity()), m_rightFront.getSelectedSensorVelocity());
    }

    private double quadratureUnitsToMeters(double quadratureCounts) {
        double motorRotations = (double)quadratureCounts / Constants.Drivetrain.kCountsPerRev;
        double wheelRotations = motorRotations / Constants.Drivetrain.kGearRatio;
        return wheelRotations * (2 * Math.PI * Constants.Drivetrain.kWheelRadiusMeters);
    }
}
