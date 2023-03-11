// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.MeasurementConstants.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.ArmConstants.*;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  private DoubleSolenoid grabberPiston = new DoubleSolenoid(30, PneumaticsModuleType.REVPH, 0, 1);
  /** Creates a new Arm. */
  public Arm() {
    lShoulderMotor.setInverted(false);
    rShoulderMotor.setInverted(true);
    rElbowMotor.setInverted(false);
    lElbowMotor.setInverted(true);

    grabberPiston.set(kReverse);
  }

  @Override
  public void periodic() {
    // Shuffleboard.getTab("Commands").putNumber( "ElbowAngle", getElbowAngle());
    // Shuffleboard.getTab("Commands").putNumber( "ShoulderAngle", getShoulderAngle());
    
    // This method will be called once per scheduler run
  }

  public void GrabGp(Boolean tf) {
    grabberPiston.set(tf ? kReverse : kForward);
  }

  public void setElbowMotors(Double speed) {
    var safe = Math.abs(speed) < kElbowMaxSpeed ? true : new Error("Elbow speed outside of acceptable range");
    lElbowMotor.set(speed);
    rElbowMotor.set(speed);
  }

  public void setShoulderMotors(Double speed) {
    var safe = Math.abs(speed) < kShoulderMaxSpeed ? true : new Error("Shoulder speed outside of acceptable range");
    lShoulderMotor.set(speed);
    rShoulderMotor.set(speed);
  }

  public Double[] getFKArmPos() {
    var hypoteneuse = Math.sqrt(kShoulderLengthSquared + kElbowLengthSquared - (2 * kShoulderLength * kElbowLength * Math.cos(Math.PI / 180 * getElbowAngle())));
    var angleElbowShoulderGrabber = Math.acos((kShoulderLengthSquared + Math.pow(hypoteneuse, 2) - kElbowLengthSquared) / (2 * kShoulderLength * hypoteneuse));
    var angleAxisShoulderGrabber = Math.PI / 2 - (getShoulderAngle() * Math.PI / 180) - angleElbowShoulderGrabber;
    var xpos = hypoteneuse * (Math.cos(angleAxisShoulderGrabber));
    var ypos = hypoteneuse * (Math.sin(angleAxisShoulderGrabber));
    return new Double[]{xpos,ypos};
  }

  public double getIKShoulder(Double x, Double y) {
    var x2 = Math.pow(x, 2);
    var y2 = Math.pow(y, 2);
    var hypoteneuse = Math.sqrt(x2 + y2);
    var angle = 90 - (180/Math.PI) * (
      Math.atan(y/Math.abs(x))  + Math.acos(
        (Math.pow(kShoulderLength, 2) + x2 + y2 - Math.pow(kElbowLength, 2)) 
          / (2 * kShoulderLength * hypoteneuse)
          )
        );
    if (Math.abs(angle) > kShoulderMaxAngle) return getShoulderAngle(); 
    return angle;
  }

  public double getIKElbow(Double x, Double y) {
    var x2 = Math.pow(x, 2);
    var y2 = Math.pow(y, 2);
    var angle = Math.copySign((180 / Math.PI) * Math.acos(
      (Math.pow(kShoulderLength, 2) + Math.pow(kElbowLength, 2) - x2 - y2) / (2 * kShoulderLength * kElbowLength)
    ), Math.signum(x));
    if (Math.abs(angle) > kElbowMaxAngle) return getShoulderAngle();
    return angle;

  }


  public double getElbowAngle() {
    var pos = (elbowEncoder.getAbsolutePosition()*360 - kElbowEncoderOffset - 180) % 360 + 180;
    pos = pos > 180 ? pos-360: pos;
    return pos;
  }
  
  public double getShoulderAngle() {
    return (shoulderEncoder.getAbsolutePosition()*360 - kShoulderEncoderOffset- 180) % 360 + 180;
  }
}
