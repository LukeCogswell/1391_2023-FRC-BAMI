// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.CANConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax lMotor = new CANSparkMax(kLeftCollectorMotorID, MotorType.kBrushless);
  private CANSparkMax rMotor = new CANSparkMax(kRightCollectorMotorID, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetCollector(Integer mode, Double speed) {
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
      default:
        lMotor.set(speed);
        rMotor.set(-speed);
    }
  }
  
}
