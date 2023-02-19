// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.CANConstants.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private DoubleSolenoid extensionPiston = new DoubleSolenoid(30, PneumaticsModuleType.REVPH, 0, 1);
  private DoubleSolenoid collectionPiston = new DoubleSolenoid(30, PneumaticsModuleType.REVPH, 2, 3);
  private CANSparkMax lMotor = new CANSparkMax(kLeftCollectorMotorID, MotorType.kBrushless);
  private CANSparkMax rMotor = new CANSparkMax(kRightCollectorMotorID, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {
    collectionPiston.set(kReverse);
    extensionPiston.set(kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void CollectorOut(Boolean tf) {
    extensionPiston.set(tf ? kReverse : kForward);
  }
  
  public void PivotIn(Boolean tf) {
    collectionPiston.set(tf ? kReverse : kForward);
  }

  public Boolean isExtended() {
    var state = collectionPiston.get() == kForward ? false : true;
    return state;
  }

  public void SetCollector(Integer mode, Double speed) {
    // if (extensionPiston.get() == kReverse) {
    //   return;
    // }
    //0 -> pull in , 1 -> clockwise, -1 -> counter clockwise
    switch(mode) {
      case 1:
        lMotor.set(-speed);
        rMotor.set(-speed);
        break;
      case -1:
        lMotor.set(speed);
        rMotor.set(speed);
        break;
      case 2:
        lMotor.set(-speed);
        rMotor.set(speed);
      default:
        lMotor.set(speed);
        rMotor.set(-speed);
    }
  }
  
}
