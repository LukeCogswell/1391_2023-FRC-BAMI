// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Stinger extends SubsystemBase {
//   private DutyCycleEncoder stingerEncoder = new DutyCycleEncoder(0);
//   private CANSparkMax spoolMotor = new CANSparkMax(19, MotorType.kBrushless);
  
//   /** Creates a new Stinger. */
//   public Stinger() {

//   }

//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("Stinger Angle", stingerEncoder.getAbsolutePosition());
//     SmartDashboard.putNumber("Stinger Pos", spoolMotor.getEncoder().getPosition());
//     // This method will be called once per scheduler run
//   }

//   public double getStingerAngle() {
//     return stingerEncoder.getAbsolutePosition()*360 - 180; //0-1 to degrees -180 to 180
//   }
  
//   public void setSpoolMotor(Double power) {
//     spoolMotor.set(power);
//   }

// }
