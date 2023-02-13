// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.MeasurementConstants.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public CANSparkMax lElbowMotor = new CANSparkMax(kLeftElbowMotorID, MotorType.kBrushless);
  private CANSparkMax rElbowMotor = new CANSparkMax(kRightElbowMotorID, MotorType.kBrushless);
  private CANSparkMax lShoulderMotor = new CANSparkMax(kLeftShoulderMotorID, MotorType.kBrushless);
  private CANSparkMax rShoulderMotor = new CANSparkMax(kRightShoulderMotorID, MotorType.kBrushless);
  private DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(1);
  private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(0);
  /** Creates a new Arm. */
  public Arm() {
    lShoulderMotor.setInverted(false);
    rShoulderMotor.setInverted(true);
    rElbowMotor.setInverted(false);
    lElbowMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber( "ElbowAngle", getElbowAngle());
    SmartDashboard.putNumber( "ShoulderAngle", getShoulderAngle());
    
    // This method will be called once per scheduler run
  }

  public void setElbowMotors(Double speed) {
    lElbowMotor.set(speed);
    rElbowMotor.set(speed);
  }

  public void setShoulderMotors(Double speed) {
    lShoulderMotor.set(speed);
    rShoulderMotor.set(speed);
  }

  public double getElbowAngle() {
    return elbowEncoder.getAbsolutePosition() - kElbowEncoderOffset;
  }
  
  public double getShoulderAngle() {
    return -(shoulderEncoder.getAbsolutePosition() - kShoulderEncoderOffset);
  }
}
