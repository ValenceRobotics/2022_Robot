// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private CANSparkMax leftClimb;
  // on flash: current limit = 40, reverse soft limit = 4, forward soft limit = 275, invert =false, +sp = up, -sp = down
  private CANSparkMax rightClimb;
  // on flash: current limit = 40, reverse soft limit = 0, forward soft limit = 262, invert = false, +sp = down, -sp = up
  private RelativeEncoder lc_enc;
  private RelativeEncoder rc_enc;

  public ClimberSubsystem() {
    leftClimb = new CANSparkMax(Constants.Climber.kLeftClimb, MotorType.kBrushless);
    rightClimb = new CANSparkMax(Constants.Climber.kRightClimb, MotorType.kBrushless);
    lc_enc = leftClimb.getEncoder();
    rc_enc = rightClimb.getEncoder();
    resetEncoders();

    leftClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setIdleMode(IdleMode.kBrake);
    leftClimb.setSmartCurrentLimit(50);
    rightClimb.setSmartCurrentLimit(50);

    leftClimb.setSoftLimit(SoftLimitDirection.kForward, 269);
    leftClimb.setSoftLimit(SoftLimitDirection.kReverse, 2);
    leftClimb.setInverted(false);
    rightClimb.setInverted(true);
    rightClimb.setSoftLimit(SoftLimitDirection.kForward, 264);
    rightClimb.setSoftLimit(SoftLimitDirection.kReverse, 2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("left enc: " + getLeftEnc() + "; right enc: " + getRightEnc());
  }

  public void setSpeed(double speed){
    leftClimb.set(speed);
    rightClimb.set(speed);
  }

  public double getLeftEnc(){
    return lc_enc.getPosition();
  }

  public void resetEncoders(){
    lc_enc.setPosition(0);
    rc_enc.setPosition(0);
  }

  public double getRightEnc(){
    return rc_enc.getPosition();
  }
}
